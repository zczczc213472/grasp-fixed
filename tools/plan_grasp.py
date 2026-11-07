"""头部全局观察 -> 规划落点 -> 移动一次 -> 腕部深度精扫 -> 一次下降抓取。

分工：
  头部彩色（鱼眼等距模型）只负责"往哪边看"，输出粗定位；
  腕部深度负责精确三维，决定落点；
  中间只移动一次，不做低空横移。
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
# 实测可达区（垂直抓取，z 0.90-0.95）
X_MIN, X_MAX = 0.62, 0.88
Y_MIN, Y_MAX = -0.03, 0.30
SCAN_PT = (0.72, 0.12)          # 可达区中心的默认扫描位


from grasp_client import connect, call


def main():
    t_all = time.time()
    p, boot = connect()
    print("启动 %.1f s" % (boot["boot_ms"] / 1000.0))

    st = call(p, {"cmd": "state"})[1]
    print("腕部当前位置 %s" % (st["mount"],))

    # ---------- ① 头部全局观察 ----------
    dt, ov = call(p, {"cmd": "overview", "table_z": 0.712})
    if not ov.get("ok"):
        print("头部观察失败：%s" % ov.get("error"))
        call(p, {"cmd": "quit"}); p.wait(timeout=10); return
    print("\n① 头部观察 %.0f ms  候选 %d 个" % (dt, ov["n"]))
    zone_c = [c for c in ov["cands"] if in_zone(c["xy"][0], c["xy"][1])]
    for c in ov["cands"][:5]:
        mark = "★在可达区" if in_zone(c["xy"][0], c["xy"][1]) else ""
        print("   xy=(%.3f, %.3f) 约 %.1f×%.1f cm  %s" % (c["xy"][0], c["xy"][1],
                                                          c["size_cm"][0], c["size_cm"][1], mark))

    if zone_c:
        tx, ty = zone_c[0]["xy"]
        print("   → 头部指示目标在可达区 (%.3f, %.3f)" % (tx, ty))
    else:
        tx, ty = SCAN_PT
        print("   → 头部没有落在可达区的明确目标，按默认扫描位 (%.2f, %.2f)" % (tx, ty))

    # ---------- ② 移动一次 ----------
    dt, mv = call(p, {"cmd": "cart", "x": tx, "y": ty, "z": HIGH, "speed": 0.15})
    print("② 移动到扫描位 %.0f ms  ok=%s err=%s mm" % (dt, mv.get("ok"), mv.get("err_mm")))
    if not mv.get("ok"):
        print("   移动失败（超出可达），放弃")
        call(p, {"cmd": "quit"}); p.wait(timeout=10); return

    # ---------- ③ 腕部深度精扫 ----------
    call(p, {"cmd": "grip", "w": 0.105, "v": 0.07, "eff": 3})
    dt, f = call(p, {"cmd": "find", "frames": 5})
    print("③ 深度精扫 %.0f ms  桌面 z=%.3f  检出 %d 个" % (dt, f["table_z"], f["n"]))
    for o in f["objects"]:
        print("   xy=(%.3f, %.3f) 高 %.1fcm 尺寸 %s %s" % (o["xy"][0], o["xy"][1], o["h_cm"],
                                                         o["size_cm"], "★" if in_zone(o["xy"][0], o["xy"][1]) else ""))
    if not f["n"]:
        print("   扫描范围内没有物体")
        call(p, {"cmd": "quit"}); p.wait(timeout=10); return

    # ---------- ④ 挑目标 + 可达性 ----------
    obj = None
    for o in f["objects"]:
        sz = o["size_cm"]
        if not (2.0 <= o["h_cm"] <= 12.0) or not in_zone(o["xy"][0], o["xy"][1]):
            continue
        if sz[0] < 2.5 or sz[1] < 2.5:        # 太小抓不住

            continue
        mz = o["top_z"] + TOOL_LEN - GRIP_DEPTH
        if call(p, {"cmd": "ik", "x": o["xy"][0], "y": o["xy"][1], "z": mz})[1].get("ok"):
            obj = o
            break
    if obj is None:
        print("   没有既在可达区、又解得出逆解的目标")
        call(p, {"cmd": "quit"}); p.wait(timeout=10); return

    ox, oy, top = obj["xy"][0], obj["xy"][1], obj["top_z"]
    mount_z = top + TOOL_LEN - GRIP_DEPTH
    print("④ 目标 xy=(%.3f, %.3f) 顶面 %.3f → 腕部 %.3f" % (ox, oy, top, mount_z))

    call(p, {"cmd": "cart", "x": ox, "y": oy, "z": HIGH, "speed": 0.10})

    # ---------- ⑤ 一次下降 + 合爪 ----------
    t_g = time.time()
    dt, mv = call(p, {"cmd": "cart", "x": ox, "y": oy, "z": mount_z, "speed": 0.15})
    print("⑤ 单次下降 %.0f ms  误差 %s mm" % (dt, mv.get("err_mm")))
    dt, g = call(p, {"cmd": "grip", "w": 0.012, "v": 0.10, "eff": 2, "blocking": False, "settle": 1.0})
    got = g["grasped"]
    print("⑥ 合爪     %.0f ms  宽度 %.4f  力矩 %.1f → %s"
          % (dt, g["width"], g["effort"], "夹住" if got else "没夹住"))
    if got:
        call(p, {"cmd": "cart", "x": ox, "y": oy, "z": min(mount_z + 0.12, 1.14), "speed": 0.06})
        print("⑦ 已抬升")
    print("\n抓取段耗时 %.1f s ／ 全流程 %.1f s" % (time.time() - t_g, time.time() - t_all))

    call(p, {"cmd": "quit"})
    p.wait(timeout=10)


if __name__ == "__main__":
    main()
