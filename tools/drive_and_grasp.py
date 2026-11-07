"""底盘小步前进把目标拉进可达区，然后一次抓取。

闭环：扫 -> 看目标是否可达 -> 不可达就前进一小步 -> 再扫，最多 4 轮。
"""
import base64
import json
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
AGENT = ROOT / "robot_exec.py"
HOST = "galbot@192.168.167.46"
SOCK = "/tmp/kitt-console-live.sock"

TOOL_LEN = 0.189
GRIP_DEPTH = 0.025
HIGH = 1.06
REACH_X = 0.845          # 超过这个 x 基本抓不到
STEP_V, STEP_T = 0.035, 1.6      # 每次前进约 5-6 cm

_ID = [0]


from grasp_client import connect, call


def pick(objs):
    """挑一个像样的方块：高度 2.5-12cm，两个方向都不小于 2.5cm。"""
    for o in objs:
        if 2.5 <= o["h_cm"] <= 12.0 and o["size_cm"][0] >= 2.5 and o["size_cm"][1] >= 2.5:
            return o
    return None


def main():
    t_all = time.time()
    p, boot = connect()
    print("启动 %.1f s" % (boot.get("boot_ms", 0) / 1000.0))

    st = call(p, {"cmd": "state"})[1]
    # 手臂先抬到安全高度再让底盘动
    call(p, {"cmd": "cart", "x": st["mount"][0], "y": st["mount"][1], "z": HIGH, "speed": 0.12})
    call(p, {"cmd": "grip", "w": 0.105, "v": 0.07, "eff": 3})

    target = None
    for it in range(4):
        dt, f = call(p, {"cmd": "find", "frames": 5})
        objs = f.get("objects", [])
        print("\n第 %d 次扫描（%.0f ms）：%d 个物体" % (it + 1, dt, len(objs)))
        for o in objs[:5]:
            print("   xy=(%.3f, %.3f) 高 %.1fcm 尺寸 %s" % (o["xy"][0], o["xy"][1],
                                                          o["h_cm"], o["size_cm"]))
        cand = pick(objs)
        if cand is None:
            print("   没有像样的方块")
            break
        x, y = cand["xy"]
        print("   选中 xy=(%.3f, %.3f)" % (x, y))
        if x <= REACH_X:
            target = cand
            print("   ✓ 已进入可达区")
            break
        d = x - (REACH_X - 0.03)
        dur = min(2.2, max(0.8, d / STEP_V))
        print("   超出可达 %.1f cm → 前进 %.1f s（约 %.1f cm）" % ((x - REACH_X) * 100, dur, STEP_V * dur * 100))
        dt2, mv = call(p, {"cmd": "base", "vx": STEP_V, "dur": dur})
        print("   底盘移动 %.0f ms  status=%s" % (dt2, mv.get("status")))
        time.sleep(0.5)

    if target is None:
        print("\n没能把目标弄进可达区，放弃")
        call(p, {"cmd": "quit"}); p.wait(timeout=10); return

    # ---- 抓取 ----
    ox, oy, top = target["xy"][0], target["xy"][1], target["top_z"]
    mount_z = top + TOOL_LEN - GRIP_DEPTH
    print("\n目标 xy=(%.3f, %.3f) 顶面 %.3f → 腕部 %.3f" % (ox, oy, top, mount_z))

    ik = call(p, {"cmd": "ik", "x": ox, "y": oy, "z": mount_z})[1]
    print("逆解：%s" % ik.get("status"))
    if not ik.get("ok"):
        call(p, {"cmd": "quit"}); p.wait(timeout=10); return

    t_g = time.time()
    dt, a = call(p, {"cmd": "cart", "x": ox, "y": oy, "z": HIGH, "speed": 0.15})
    print("① 高空对准 %.0f ms 误差 %s mm" % (dt, a.get("err_mm")))

    # 到位后复核：目标是否真的在工具轴正下方（相机偏轴 7cm，必须闭环校正）
    aligned = False
    for k in range(3):
        _, st2 = call(p, {"cmd": "state"})
        mx, my = st2["mount"][0], st2["mount"][1]
        _, f2 = call(p, {"cmd": "find", "frames": 4})
        near = None
        best = 9.9
        for o in f2.get("objects", []):
            d = ((o["xy"][0] - mx) ** 2 + (o["xy"][1] - my) ** 2) ** 0.5
            if d < best:
                best, near = d, o
        if near is None:
            print("   复核：视野里找不到目标")
            break
        dx = near["xy"][0] - mx
        dy = near["xy"][1] - my
        print("   复核 %d：最近目标 xy=(%.3f, %.3f) 偏离工具轴 (%.1f, %.1f) cm"
              % (k + 1, near["xy"][0], near["xy"][1], dx * 100, dy * 100))
        if abs(dx) < 0.012 and abs(dy) < 0.012:
            aligned = True
            print("   ✓ 已对准")
            ox, oy, top = near["xy"][0], near["xy"][1], near["top_z"]
            mount_z = top + TOOL_LEN - GRIP_DEPTH
            break
        ox, oy = mx + dx, my + dy
        call(p, {"cmd": "cart", "x": ox, "y": oy, "z": HIGH, "speed": 0.12})
    if not aligned:
        raise RuntimeError("目标复核未对准，中止下降")
    dt, a = call(p, {"cmd": "cart", "x": ox, "y": oy, "z": mount_z, "speed": 0.15})
    print("② 一次下降 %.0f ms 误差 %s mm" % (dt, a.get("err_mm")))
    dt, g = call(p, {"cmd": "grip", "w": 0.012, "v": 0.10, "eff": 2,
                     "blocking": False, "settle": 1.0})
    got = g["grasped"]
    print("③ 合爪     %.0f ms 宽度 %.4f 力矩 %.1f → %s"
          % (dt, g["width"], g["effort"], "夹住" if got else "没夹住"))
    if got:
        call(p, {"cmd": "cart", "x": ox, "y": oy, "z": min(mount_z + 0.14, 1.14), "speed": 0.08})
        gg = call(p, {"cmd": "state"})[1]["grip"]
        print("④ 已抬升，宽度 %.4f 力矩 %.1f" % (gg["width"], gg["effort"]))

    print("\n抓取段 %.1f s ／ 全程 %.1f s" % (time.time() - t_g, time.time() - t_all))
    call(p, {"cmd": "quit"})
    p.wait(timeout=10)


if __name__ == "__main__":
    main()
