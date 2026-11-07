"""机器人侧常驻执行器：一次 init，之后按 JSONL 收发命令。

命令（每行一个 JSON，返回一行 JSON）：
  {"id":1,"cmd":"state"}
  {"id":2,"cmd":"cart","x":0.77,"y":0.16,"z":1.02,"tilt":0}
  {"id":3,"cmd":"joints","q":[7 个关节角]}
  {"id":4,"cmd":"grip","w":0.09,"v":0.06,"eff":3}
  {"id":5,"cmd":"depth"}            -> 返回深度统计（不下发图像）
  {"id":6,"cmd":"quit"}
"""
from __future__ import annotations

import json
import sys
import time

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

import galbot_sdk as g

S = g.SensorType
X_AXIS_BASE = np.array([0.0, 0.0, -1.0])
TOOL_LEN_M = 0.189          # 安装帧 -> 指尖（实测）


def q2R(q):
    x, y, z, w = [float(v) for v in q]
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R2q(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = (tr + 1.0) ** 0.5 * 2
        w, x, y, z = .25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, .25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, .25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, .25 * s
    return [x, y, z, w]


def tool_orientation(tilt_deg: float):
    """工具轴：0° 垂直向下；tilt 为绕 x 轴倾斜，向 -y 方向倒。"""
    th = np.radians(float(tilt_deg))
    xa = np.array([0.0, -np.sin(th), -np.cos(th)])
    xa /= np.linalg.norm(xa)
    ya = np.array([1.0, 0.0, 0.0])
    za = np.cross(xa, ya)
    za /= np.linalg.norm(za)
    ya = np.cross(za, xa)
    return np.stack([xa, ya, za], axis=1)


class Exec:
    def __init__(self):
        t0 = time.time()
        self.r = g.GalbotRobot.get_instance(g.MachineType.G1)
        # 只 init 一次，并且直接带深度相机（同进程二次 init 不会真正启用传感器）
        try:
            self.sensor_ok = bool(self.r.init(
                enable_sensor_set={S.LEFT_ARM_DEPTH_CAMERA, S.HEAD_LEFT_CAMERA},
                enable_sync_mode=False))
        except Exception:
            self.sensor_ok = False
            raise RuntimeError("SDK 初始化失败；退出，避免同进程重复 init")
        self.sensor_ready = self.sensor_ok
        self.m = g.GalbotMotion.get_instance(g.MachineType.G1)
        self.m.init()
        self.boot_ms = int((time.time() - t0) * 1000)

    # ---------- 基础 ----------
    def joints(self):
        js = self.m.get_chain_joint_state()
        return {k: [round(float(v), 5) for v in val] for k, val in js.items()}

    def mount(self, js=None):
        js = js or self.m.get_chain_joint_state()
        st, mo = self.m.forward_kinematics("left_arm_end_effector_mount_link", "base_link", js)
        if mo is None or len(mo) < 7:
            return None
        return np.array(mo[:3]), q2R(mo[3:])

    def state(self):
        js = self.m.get_chain_joint_state()
        mp = self.mount(js)
        gs = self.r.get_gripper_state("left_gripper")
        out = {"ok": True, "joints": {k: [round(float(v), 4) for v in val] for k, val in js.items()},
               "grip": {"width": round(float(gs.width), 4), "effort": round(float(gs.effort), 2),
                        "moving": bool(gs.is_moving)}}
        if mp is not None:
            p, R = mp
            out["mount"] = [round(float(v), 4) for v in p]
            out["tool_axis"] = [round(float(v), 3) for v in (R @ np.array([1.0, 0, 0]))]
        return out

    # 左臂 D405 实测内参（1280x720），用于接口偶发拿不到 K 时兜底
    K_FALLBACK = [657.416, 0.0, 636.383, 0.0, 656.949, 356.112, 0.0, 0.0, 1.0]

    def ensure_sensor(self):
        """深度相机在启动时就已启用；这里只做可用性检查。"""
        if not self.sensor_ready:
            raise RuntimeError("深度相机未启用（可能被其他进程占用，如观察台 8410）")
        return True

    def _depth(self):
        self.ensure_sensor()
        d = self.r.get_depth_data(S.LEFT_ARM_DEPTH_CAMERA)
        if not d or "width" not in d:
            raise RuntimeError("取不到深度数据（相机被占用或未就绪）")
        return d

    def intrinsics(self):
        try:
            d = self.r.get_camera_intrinsic(S.LEFT_ARM_DEPTH_CAMERA)
            if isinstance(d, dict) and "K" in d and len(d["K"]) >= 6:
                return list(d["K"])
        except Exception:
            pass
        return list(self.K_FALLBACK)

    # ---------- 命令 ----------
    def cmd_state(self, req):
        return self.state()

    def cmd_joints(self, req):
        q = [float(v) for v in req["q"]]
        current = self.m.get_chain_joint_state()["left_arm"]
        if len(q) != 7 or not np.all(np.isfinite(q)):
            return {"ok": False, "error": "Expected 7 finite joint angles"}
        if max(abs(float(np.degrees(b-a))) for a,b in zip(current,q)) > 45:
            return {"ok": False, "error": "Joint delta exceeds 45 degrees"}
        t0 = time.time()
        st = self.r.set_joint_positions(q, joint_groups=["left_arm"], is_blocking=True,
                                        speed_rad_s=float(req.get("speed", 0.08)),
                                        timeout_s=float(req.get("timeout", 30.0)))
        out = self.state()
        out["status"] = str(st).replace("ControlStatus.", "")
        out["ok"] = out["status"] == "SUCCESS"
        out["move_ms"] = int((time.time() - t0) * 1000)
        return out

    def cmd_cart(self, req):
        p_des = np.array([float(req["x"]), float(req["y"]), float(req["z"])])
        R_des = tool_orientation(float(req.get("tilt", 0.0)))
        tgt = [float(v) for v in list(p_des) + R2q(R_des)]
        js = self.m.get_chain_joint_state()
        seed = [float(v) for v in js["left_arm"]]
        t0 = time.time()
        st, sol = self.m.inverse_kinematics(tgt, ["left_arm"], "left_arm_end_effector_mount_link",
                                            "base_link", {"left_arm": seed}, True)
        ik_ms = int((time.time() - t0) * 1000)
        if not (sol and sol.get("left_arm")):
            return {"ok": False, "ik": str(st).replace("MotionStatus.", ""), "ik_ms": ik_ms}
        q = [float(v) for v in sol["left_arm"]]
        delta = max(abs(float(np.degrees(b - a))) for a, b in zip(seed, q))
        if delta > 45:
            return {"ok": False, "error": "IK joint delta exceeds 45 degrees", "max_delta_deg": delta}
        t0 = time.time()
        s2 = self.r.set_joint_positions(q, joint_groups=["left_arm"], is_blocking=True,
                                        speed_rad_s=float(req.get("speed", 0.08)),
                                        timeout_s=float(req.get("timeout", 30.0)))
        out = self.state()
        out["status"] = str(s2).replace("ControlStatus.", "")
        out["ok"] = out["status"] == "SUCCESS"
        out["ik_ms"] = ik_ms
        out["move_ms"] = int((time.time() - t0) * 1000)
        mp = self.mount()
        if mp is not None:
            out["err_mm"] = round(float(np.linalg.norm(mp[0] - p_des)) * 1000, 1)
        out["ok"] = out["ok"] and out.get("err_mm", float("inf")) <= 10
        return out

    def cmd_grip(self, req):
        t0 = time.time()
        st = self.r.set_gripper_command("left_gripper", float(req.get("w", 0.09)),
                                        velocity_mps=float(req.get("v", 0.05)),
                                        effort=float(req.get("eff", 3)),
                                        is_blocking=bool(req.get("blocking", False)))
        time.sleep(float(req.get("settle", 0.0)))
        gs = self.r.get_gripper_state("left_gripper")
        return {"ok": str(st).split(".")[-1] == "SUCCESS", "status": str(st).replace("ControlStatus.", ""),
                "width": round(float(gs.width), 4), "effort": round(float(gs.effort), 2),
                "ms": int((time.time() - t0) * 1000)}

    def cmd_depth(self, req):
        t0 = time.time()
        d = self._depth()
        w, h, unit = d["width"], d["height"], d["depth_scale"]
        raw = np.frombuffer(bytes(d["data"]), dtype="<u2").reshape(h, w).astype(np.float32) / unit
        msk = (raw > 0.06) & (raw < 0.80)
        return {"ok": True, "w": w, "h": h, "valid_pct": round(100.0 * msk.sum() / (w * h), 1),
                "z_p50": round(float(np.median(raw[msk])), 3) if msk.sum() else None,
                "ms": int((time.time() - t0) * 1000)}

    def cmd_base(self, req):
        """底盘小步移动：给速度 + 持续时间，结束立刻停。"""
        vx = float(req.get("vx", 0.0))
        vy = float(req.get("vy", 0.0))
        wz = float(req.get("wz", 0.0))
        dur = float(req.get("dur", 0.0))
        t0 = time.time()
        st = self.r.set_base_velocity([vx, vy, 0.0], [0.0, 0.0, wz], duration_s=dur)
        try:
            time.sleep(max(0.0, dur) + 0.25)
        finally:
            self.r.stop_base()
        time.sleep(0.4)
        try:
            vel = self.r.get_base_velocity()
        except Exception:
            vel = None
        return {"ok": str(st).split(".")[-1] == "SUCCESS", "status": str(st).replace("ControlStatus.", ""),
                "vx": vx, "dur": dur, "vel_after": vel,
                "ms": int((time.time() - t0) * 1000)}

    def cmd_overview(self, req):
        """头部彩色全局观察：找出桌面上的候选物体，反投影到 base 平面。"""
        if cv2 is None:
            return {"ok": False, "error": "cv2 不可用"}
        t0 = time.time()
        table_z = float(req.get("table_z", 0.712))
        dur = float(req.get("dur", 0.0))

        img = None
        for _ in range(5):
            rgb = self.r.get_rgb_data(S.HEAD_LEFT_CAMERA)
            if rgb and rgb.get("data"):
                img = cv2.imdecode(np.frombuffer(bytes(rgb["data"]), np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    break
        if img is None:
            return {"ok": False, "error": "头部彩色取不到图"}
        t_img = time.time()

        js = self.m.get_chain_joint_state()
        st, cf = self.m.forward_kinematics("head_left_camera_color_optical_frame", "base_link", js)
        if cf is None or len(cf) < 7:
            return {"ok": False, "error": "头部相机位姿取不到"}
        p_c = np.array(cf[:3])
        R_c = q2R(cf[3:7])

        K = [412.7, 0.0, 668.2, 0.0, 412.7, 485.2, 0.0, 0.0, 1.0]
        try:
            d = self.r.get_camera_intrinsic(S.HEAD_LEFT_CAMERA)
            if isinstance(d, dict) and "K" in d and len(d["K"]) >= 6:
                K = list(d["K"])
        except Exception:
            pass
        fx, fy, cx0, cy0 = K[0], K[4], K[2], K[5]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 桌面是深色布，物体通常更亮或更鲜艳
        sat = hsv[:, :, 1].astype(np.int16)
        val = hsv[:, :, 2].astype(np.int16)
        mask = ((sat > 70) & (val > 55)) | (val > 165)
        mask = cv2.morphologyEx(mask.astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
        H, W = img.shape[:2]
        cands = []
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < 400:
                continue
            x0 = int(stats[i, cv2.CC_STAT_LEFT]); y0 = int(stats[i, cv2.CC_STAT_TOP])
            ww = int(stats[i, cv2.CC_STAT_WIDTH]); hh = int(stats[i, cv2.CC_STAT_HEIGHT])
            if ww > W * 0.5 or hh > H * 0.5:
                continue                      # 太大的是桌面/机身
            u, v = cents[i]
            # 鱼眼等距模型：theta = r / f
            dx = u - cx0
            dy = v - cy0
            r_px = float(np.hypot(dx, dy))
            theta = r_px / fx
            if theta > np.radians(88):
                continue
            phi = float(np.arctan2(dy, dx))
            dvec = np.array([np.sin(theta) * np.cos(phi),
                             np.sin(theta) * np.sin(phi),
                             np.cos(theta)])
            dirv = R_c @ dvec
            if abs(dirv[2]) < 1e-4:
                continue
            s_ = (table_z - p_c[2]) / dirv[2]
            if not (0.1 < s_ < 2.5):
                continue
            P = p_c + dirv * s_
            # 由像素尺寸估算实际尺寸（用该角度下的等效焦距）
            eff = fx / max(0.25, np.cos(theta))
            real_w = ww * s_ / eff
            real_h = hh * s_ / eff
            cands.append({"area": area, "uv": [round(float(u), 1), round(float(v), 1)],
                          "xy": [round(float(P[0]), 3), round(float(P[1]), 3)],
                          "size_cm": [round(real_w * 100, 1), round(real_h * 100, 1)],
                          "dist_m": round(float(s_), 2)})
        cands.sort(key=lambda c: -c["area"])
        return {"ok": True, "n": len(cands), "table_z": table_z,
                "img_ms": int((t_img - t0) * 1000),
                "ms": int((time.time() - t0) * 1000),
                "cands": cands[:10]}

    @staticmethod
    def _gripper_mask(Pb, p_m, R_m):
        """把实测的夹爪包围盒投到 base 系，返回"该点属于夹爪"的布尔掩码。

        盒子（安装帧）：X 0.05~0.20，Y ±0.06，Z -0.03~0.03
        """
        loc = (Pb - p_m) @ R_m
        return ((loc[:, 0] > 0.045) & (loc[:, 0] < 0.205) &
                (np.abs(loc[:, 1]) < 0.062) & (np.abs(loc[:, 2]) < 0.032))

    def cmd_target(self, req):
        """在可达区内找最高的一个物体：1cm 高度图 -> 平滑 -> 取峰 -> 加权重心。

        不做连通域，避免稀疏点云下簇形状抖动。
        """
        t0 = time.time()
        nf = int(req.get("frames", 6))
        xmin = float(req.get("xmin", 0.55)); xmax = float(req.get("xmax", 0.88))
        ymin = float(req.get("ymin", -0.10)); ymax = float(req.get("ymax", 0.32))

        ext = self.r.get_sensor_extrinsic(S.LEFT_ARM_DEPTH_CAMERA, "base_link")[0][:7]
        t = np.array(ext[:3], float); R = q2R(ext[3:7])
        K = self.intrinsics(); fx, fy, cx, cy = K[0], K[4], K[2], K[5]

        frames = []
        for _ in range(max(2, nf)):
            d = self._depth()
            w, h, unit = d["width"], d["height"], d["depth_scale"]
            frames.append(np.frombuffer(bytes(d["data"]), dtype="<u2").reshape(h, w).astype(np.float32) / unit)
        stk = np.stack(frames, 0); stk[stk <= 0] = np.nan
        med = np.nan_to_num(np.nanmedian(stk, 0), nan=0.0)

        msk = (med > 0.06) & (med < 0.80)
        vs, us = np.nonzero(msk); zz = med[msk]
        P = np.stack([(us - cx) * zz / fx, (vs - cy) * zz / fy, zz], axis=1)
        Pb = (R @ P.T).T + t

        mp = self.mount()
        if mp is None:
            return {"ok": False, "error": "no mount"}
        p_m = mp[0]
        lat0 = np.linalg.norm(Pb[:, :2] - p_m[:2], axis=1)
        far = lat0 > 0.12
        zpool = Pb[far][:, 2] if far.sum() > 2000 else Pb[:, 2]
        lo = max(0.45, float(p_m[2]) - 0.45); hi = float(p_m[2]) - 0.04
        hist, edges = np.histogram(zpool, bins=np.arange(lo, hi, 0.01))
        if hist.sum() == 0:
            return {"ok": False, "error": "no table"}
        tz = float(edges[int(np.argmax(hist))] + 0.005)
        near = np.abs(Pb[:, 2] - tz) < 0.015
        for _ in range(4):
            if near.sum() < 200:
                break
            zc = float(np.median(Pb[near][:, 2]))
            near = np.abs(Pb[:, 2] - zc) < 0.010
        if near.sum() >= 200:
            tz = float(np.median(Pb[near][:, 2]))

        # 只取可达窗口内、指尖以下的点
        gm = self._gripper_mask(Pb, p_m, mp[1])
        inwin = (Pb[:, 0] > xmin) & (Pb[:, 0] < xmax) & (Pb[:, 1] > ymin) & (Pb[:, 1] < ymax) & \
                (Pb[:, 2] > tz + 0.020) & (Pb[:, 2] < tz + 0.15) & (~gm)
        if inwin.sum() < 300:
            return {"ok": False, "error": "窗口内点太少", "table_z": round(tz, 4)}

        sub = Pb[inwin]
        # 1cm 高度图
        RES = 0.01
        gx = ((sub[:, 0] - xmin) / RES).astype(int)
        gy = ((sub[:, 1] - ymin) / RES).astype(int)
        nx = int((xmax - xmin) / RES) + 1; ny = int((ymax - ymin) / RES) + 1
        hmap = np.zeros((ny, nx), np.float32)
        cnt = np.zeros((ny, nx), np.float32)
        for a, b, z in zip(gy, gx, sub[:, 2] - tz):
            if 0 <= a < ny and 0 <= b < nx:
                hmap[a, b] += z; cnt[a, b] += 1
        avg = np.where(cnt > 0, hmap / np.maximum(cnt, 1), 0)
        ker = np.ones((3, 3), np.float32) / 9.0
        try:
            import cv2
            sm = cv2.filter2D(avg, -1, ker)
        except Exception:
            sm = avg
        # 取多个局部峰（按高度排序），供上层挑"够得着"的那个
        peaks = []
        work = sm.copy()
        for _ in range(int(req.get("peaks", 3))):
            pk = np.unravel_index(int(np.argmax(work)), work.shape)
            ph = float(work[pk])
            if ph < 0.02:
                break
            px = xmin + (pk[1] + 0.5) * RES
            py = ymin + (pk[0] + 0.5) * RES
            dxy = np.linalg.norm(sub[:, :2] - np.array([px, py]), axis=1)
            sel = dxy < 0.05
            if sel.sum() >= 50:
                wgt = np.clip(sub[sel, 2] - tz, 0, None) ** 1.5
                cxy = (sub[sel, :2] * wgt[:, None]).sum(0) / max(wgt.sum(), 1e-6)
                top = float(np.percentile(sub[sel][:, 2], 97))
                peaks.append({"xy": [round(float(cxy[0]), 3), round(float(cxy[1]), 3)],
                              "peak_xy": [round(px, 3), round(py, 3)],
                              "top_z": round(top, 4),
                              "h_cm": round((top - tz) * 100, 1),
                              "n": int(sel.sum())})
            # 抹掉这个峰，避免重复
            y0 = max(0, pk[0] - 4); y1 = min(ny, pk[0] + 5)
            x0 = max(0, pk[1] - 4); x1 = min(nx, pk[1] + 5)
            work[y0:y1, x0:x1] = 0
        if not peaks:
            return {"ok": False, "error": "没有高于 2cm 的物体", "table_z": round(tz, 4)}
        out = {"ok": True, "table_z": round(tz, 4), "cands": peaks,
               "ms": int((time.time() - t0) * 1000)}
        out.update(peaks[0])
        return out

    def cmd_ik(self, req):
        p_des = np.array([float(req["x"]), float(req["y"]), float(req["z"])])
        R_des = tool_orientation(float(req.get("tilt", 0.0)))
        tgt = [float(v) for v in list(p_des) + R2q(R_des)]
        js = self.m.get_chain_joint_state()
        seed = [float(v) for v in js["left_arm"]]
        t0 = time.time()
        st, sol = self.m.inverse_kinematics(tgt, ["left_arm"], "left_arm_end_effector_mount_link",
                                            "base_link", {"left_arm": seed}, True)
        ok = bool(sol and sol.get("left_arm"))
        out = {"ok": ok, "status": str(st).replace("MotionStatus.", ""),
               "ms": int((time.time() - t0) * 1000)}
        if ok:
            d = [float(np.degrees(b - a)) for a, b in zip(seed, [float(v) for v in sol["left_arm"]])]
            out["max_delta_deg"] = round(max(abs(v) for v in d), 1)
        return out

    # ---------- 找物体：深度 -> base 点云 -> 桌面 -> 连通域 ----------
    def cmd_find(self, req):
        t0 = time.time()
        nf = int(req.get("frames", 4))
        ext = self.r.get_sensor_extrinsic(S.LEFT_ARM_DEPTH_CAMERA, "base_link")[0][:7]
        t = np.array(ext[:3], float)
        R = q2R(ext[3:7])
        K = self.intrinsics()
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]

        frames = []
        for _ in range(max(1, nf)):
            d = self._depth()
            w, h, unit = d["width"], d["height"], d["depth_scale"]
            frames.append(np.frombuffer(bytes(d["data"]), dtype="<u2").reshape(h, w).astype(np.float32) / unit)
        stk = np.stack(frames, 0)
        stk[stk <= 0] = np.nan
        med = np.nan_to_num(np.nanmedian(stk, 0), nan=0.0)

        msk = (med > 0.06) & (med < 0.80)
        vs, us = np.nonzero(msk)
        zz = med[msk]
        P = np.stack([(us - cx) * zz / fx, (vs - cy) * zz / fy, zz], axis=1)
        Pb = (R @ P.T).T + t

        # 桌面高度：只在"远离工具轴"的点上找主峰，避免夹爪自身抢峰
        mp0 = self.mount()
        if mp0 is None:
            return {"ok": False, "error": "no mount pose"}
        lat0 = np.linalg.norm(Pb[:, :2] - mp0[0][:2], axis=1)
        far = lat0 > 0.12
        zpool = Pb[far][:, 2] if far.sum() > 2000 else Pb[:, 2]
        lo = max(0.45, float(mp0[0][2]) - 0.45)
        hi = float(mp0[0][2]) - 0.04
        hist, edges = np.histogram(zpool, bins=np.arange(lo, hi, 0.01))
        if hist.sum() == 0:
            return {"ok": False, "error": "no table points"}
        tz = float(edges[int(np.argmax(hist))] + 0.005)
        near = np.abs(Pb[:, 2] - tz) < 0.015
        for _ in range(4):
            if near.sum() < 200:
                break
            zc = float(np.median(Pb[near][:, 2]))
            near = np.abs(Pb[:, 2] - zc) < 0.010
        if near.sum() >= 200:
            tz = float(np.median(Pb[near][:, 2]))

        mp = self.mount()
        if mp is None:
            return {"ok": False, "error": "no mount pose"}
        p_m = mp[0]
        rel = Pb[:, 2] - tz
        lat = np.linalg.norm(Pb[:, :2] - p_m[:2], axis=1)
        # 排除夹爪自身（工具轴附近）+ 只留桌面上方的合理高度
        # 用实测包围盒精确剔除夹爪
        gm = self._gripper_mask(Pb, p_m, self.mount()[1])
        cand = (Pb[:, 2] > tz + 0.018) & (Pb[:, 2] < tz + 0.15) & (lat < 0.32) & (~gm)

        RES = 0.006
        X0, Y0 = float(p_m[0]) - 0.20, float(p_m[1]) - 0.20
        W = H = 68
        cells = {}
        for k in np.nonzero(cand)[0]:
            a = int((Pb[k, 1] - Y0) / RES)
            b = int((Pb[k, 0] - X0) / RES)
            if 0 <= a < H and 0 <= b < W:
                cells.setdefault((a, b), []).append(k)

        seen = set()
        comps = []
        for c in list(cells):
            if c in seen:
                continue
            sc = [c]; seen.add(c); grp = []
            while sc:
                y0, x0 = sc.pop(); grp.append((y0, x0))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        n2 = (y0 + dy, x0 + dx)
                        if n2 in cells and n2 not in seen:
                            seen.add(n2); sc.append(n2)
            if len(grp) >= 5:
                comps.append(grp)

        objs = []
        for grp in comps:
            ks = [k for cell in grp for k in cells[cell]]
            if len(ks) < 400:          # 太小的簇一律当噪声
                continue
            sub = Pb[ks]
            sz = np.percentile(sub, 97, 0) - np.percentile(sub, 3, 0)
            if sz[0] > 0.16 or sz[1] > 0.16:
                continue
            if sz[0] < 0.018 or sz[1] < 0.018:   # 细长/飞点
                continue
            objs.append({"n": len(ks),
                         "xy": [round(float(v), 4) for v in np.median(sub[:, :2], 0)],
                         "top_z": round(float(np.percentile(sub[:, 2], 97)), 4),
                         "h_cm": round((float(np.percentile(sub[:, 2], 97)) - tz) * 100, 1),
                         "size_cm": [round(float(v) * 100, 1) for v in sz]})
        objs.sort(key=lambda o: -o["h_cm"])
        return {"ok": True, "table_z": round(tz, 4), "mount": [round(float(v), 4) for v in p_m],
                "n": len(objs), "objects": objs[:8], "ms": int((time.time() - t0) * 1000)}


def main():
    ex = Exec()
    print(json.dumps({"ok": True, "ready": True, "boot_ms": ex.boot_ms,
                      "depth_cam": ex.sensor_ok}), flush=True)
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                req = json.loads(line)
            except Exception:
                continue
            cmd = req.get("cmd")
            rid = req.get("id")
            if cmd == "quit":
                print(json.dumps({"id": rid, "ok": True, "bye": True}), flush=True)
                break
            try:
                t0 = time.time()
                if cmd == "state":
                    out = ex.cmd_state(req)
                elif cmd == "cart":
                    out = ex.cmd_cart(req)
                elif cmd == "joints":
                    out = ex.cmd_joints(req)
                elif cmd == "grip":
                    out = ex.cmd_grip(req)
                elif cmd == "depth":
                    out = ex.cmd_depth(req)
                elif cmd == "find":
                    out = ex.cmd_find(req)
                elif cmd == "ik":
                    out = ex.cmd_ik(req)
                elif cmd == "target":
                    out = ex.cmd_target(req)
                elif cmd == "overview":
                    out = ex.cmd_overview(req)
                elif cmd == "base":
                    out = ex.cmd_base(req)
                else:
                    out = {"ok": False, "error": "unknown cmd"}
                out["id"] = rid
                out["total_ms"] = int((time.time() - t0) * 1000)
            except Exception as exc:
                out = {"id": rid, "ok": False, "error": "%s: %s" % (type(exc).__name__, str(exc)[:200])}
            print(json.dumps(out), flush=True)
    finally:
        ex.r.destroy()


if __name__ == "__main__":
    main()
