"""一次性抓取：深度定位 -> 可达性过滤 -> 高空对准 -> 单次垂直下降 -> 合爪 -> 抬升。

核心思想：落点完全由深度决定（top_z + 指长 - 抓深），不再逐格试探。
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

TOOL_LEN = 0.189      # 安装帧 -> 指尖（实测，别用 0.176）
GRIP_DEPTH = 0.025    # 指尖落到物体顶面以下多少
HIGH = 1.06           # 高空对准高度


from grasp_client import connect, call


def main():
    p, boot = connect()
    print("启动 %.1f s（深度相机 %s）" % (boot["boot_ms"] / 1000.0, boot.get("depth_cam")))

    st = call(p, {"cmd": "state"})[1]
    print("当前腕部 %s  夹爪 %.3f m" % (st["mount"], st["grip"]["width"]))

    call(p, {"cmd": "cart", "x": st["mount"][0], "y": st["mount"][1], "z": max(HIGH, st["mount"][2]), "speed": 0.10})
    call(p, {"cmd": "grip", "w": 0.105, "v": 0.07, "eff": 3})

    # ---- 一次扫描 ----
    dt, f = call(p, {"cmd": "find", "frames": 5})
    if not f.get("ok"):
        print("扫描失败：%s" % f.get("error"))
        call(p, {"cmd": "quit"}); p.wait(timeout=10); return
    print("扫描 %.0f ms  桌面 z=%.3f  检出 %d 个" % (dt, f["table_z"], f["n"]))
    for o in f["objects"]:
        print("   xy=(%.3f, %.3f) 高 %.1fcm 尺寸 %s 点数 %d"
              % (o["xy"][0], o["xy"][1], o["h_cm"], o["size_cm"], o["n"]))

    # ---- 可达性过滤，挑第一个够得着的 ----
    obj = None
    for o in f["objects"]:
        if not (2.5 <= o["h_cm"] <= 12.0):
            continue
        mz = o["top_z"] + TOOL_LEN - GRIP_DEPTH
        _, ik_high = call(p, {"cmd": "ik", "x": o["xy"][0], "y": o["xy"][1], "z": HIGH})
        _, ik_low = call(p, {"cmd": "ik", "x": o["xy"][0], "y": o["xy"][1], "z": mz})
        print("   检查 xy=(%.3f, %.3f)：高空 %s / 抓取点 %s"
              % (o["xy"][0], o["xy"][1], ik_high.get("status"), ik_low.get("status")))
        if ik_high.get("ok") and ik_low.get("ok"):
            obj = o
            break
    if obj is None:
        print("没有既合适又够得着的目标")
        call(p, {"cmd": "quit"}); p.wait(timeout=10); return

    ox, oy, top = obj["xy"][0], obj["xy"][1], obj["top_z"]
    mount_z = top + TOOL_LEN - GRIP_DEPTH
    print("\n目标 xy=(%.3f, %.3f) 顶面 z=%.3f → 腕部降到 z=%.3f（指尖到 %.3f）"
          % (ox, oy, top, mount_z, mount_z - TOOL_LEN))

    t0 = time.time()
    dt, r = call(p, {"cmd": "cart", "x": ox, "y": oy, "z": HIGH, "speed": 0.10})
    print("① 高空对准  %.0f ms  误差 %s mm" % (dt, r.get("err_mm")))

    dt, r = call(p, {"cmd": "cart", "x": ox, "y": oy, "z": mount_z, "speed": 0.05})
    print("② 一次下降  %.0f ms  误差 %s mm（不再分步）" % (dt, r.get("err_mm")))

    dt, g = call(p, {"cmd": "grip", "w": 0.012, "v": 0.015, "eff": 2,
                     "blocking": False, "settle": 0.35})
    got = g["grasped"]
    print("③ 合爪      %.0f ms  宽度 %.4f  力矩 %.1f → %s"
          % (dt, g["width"], g["effort"], "夹住" if got else "没夹住"))

    if got:
        dt, r = call(p, {"cmd": "cart", "x": ox, "y": oy,
                         "z": min(mount_z + 0.12, 1.14), "speed": 0.06})
        gg = call(p, {"cmd": "state"})[1]["grip"]
        print("④ 抬升      %.0f ms  宽度 %.4f  力矩 %.1f" % (dt, gg["width"], gg["effort"]))
    print("\n=== 对准到夹住：%.1f s ===" % (time.time() - t0))

    call(p, {"cmd": "quit"})
    p.wait(timeout=10)


if __name__ == "__main__":
    main()
