"""Shared JSONL client. No robot commands run on import."""
import atexit
import base64
import json
import queue
import subprocess
import threading
import time
from pathlib import Path

HOST = 'galbot@192.168.167.46'
SOCK = '/tmp/kitt-console-live.sock'
ROOT = Path(__file__).resolve().parent.parent

class Channel:
    def __init__(self, process):
        self.p = process
        self.lines = queue.Queue()
        self.rid = 0
        self.failed = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        try:
            for line in self.p.stdout:
                self.lines.put(line)
        finally:
            self.lines.put(None)

    def receive(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('机器人响应超时；中止流程，不再发送运动命令')
            try:
                line = self.lines.get(timeout=remaining)
            except queue.Empty:
                raise TimeoutError('机器人响应超时；中止流程，不再发送运动命令')
            if line is None:
                raise ConnectionError('SSH 连接已断开')
            try:
                obj = json.loads(line)
            except (ValueError, TypeError):
                continue
            if isinstance(obj, dict) and predicate(obj):
                return obj

    def request(self, obj, timeout=40):
        if self.failed:
            raise RuntimeError('连接已失效，禁止继续发送命令')
        self.rid += 1
        obj = dict(obj, id=self.rid)
        try:
            self.p.stdin.write(json.dumps(obj) + '\n')
            self.p.stdin.flush()
            result = self.receive(lambda r: r.get('id') == self.rid, timeout)
            # IK failure is a normal candidate-filtering result.
            if not result.get('ok') and obj.get('cmd') != 'ik':
                raise RuntimeError('%s 失败: %s' % (obj.get('cmd'), result))
            return result
        except Exception:
            self.failed = True
            raise


def grip_result(samples, target, contact=True):
    """Require three stationary samples; effort values retain original SDK units."""
    if len(samples) < 3:
        return None
    tail = samples[-3:]
    if any(s.get('moving', True) for s in tail):
        return None
    widths = [s['width'] for s in tail]
    if max(widths) - min(widths) > 0.001:
        return None
    if contact and all(s['width'] > target + 0.003 and abs(s['effort']) > 15 for s in tail):
        return 'contact'
    if all(abs(s['width'] - target) <= 0.002 for s in tail):
        return 'empty' if contact else 'reached'
    return None


def connect():
    payload = base64.b64encode((ROOT / 'robot_exec.py').read_bytes()).decode()
    cmd = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-T']
    if Path(SOCK).exists():
        cmd += ['-o', 'ControlPath=' + SOCK]
    cmd += [HOST, 'PYTHONPATH=/data/galbot/lib python3 -u -c "import base64;exec(base64.b64decode(\'%s\'))"' % payload]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=None, text=True, bufsize=1)
    p.channel = Channel(p)
    def cleanup():
        # No gripper opening or extra motion on cleanup.
        if p.poll() is None:
            p.stdin.close()
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.terminate()
    atexit.register(cleanup)
    try:
        ready = p.channel.receive(lambda r: r.get('ready') is True, 30)
        if not ready.get('ok') or not ready.get('depth_cam'):
            raise RuntimeError('机器人或深度相机未就绪')
        return p, ready
    except Exception:
        cleanup()
        raise


def call(p, obj):
    start = time.monotonic()
    req = dict(obj)
    if req.get('cmd') != 'grip':
        result = p.channel.request(req)
    else:
        # Never wait for the SDK's blocking gripper timeout.
        req.update(blocking=False, settle=0)
        result = p.channel.request(req, timeout=5)
        target = float(req.get('w', 0.09))
        closing = target <= 0.02
        samples = []
        deadline = time.monotonic() + 10
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                p.channel.failed = True
                raise TimeoutError('夹爪未在 10 秒内稳定，停止后续抓取动作')
            state = p.channel.request({'cmd': 'state'}, timeout=min(3, remaining))
            samples.append(state['grip'])
            verdict = grip_result(samples, target, contact=closing)
            if verdict:
                result.update(samples[-1], grasped=verdict == 'contact', verdict=verdict)
                break
            time.sleep(0.1)
    return (time.monotonic() - start) * 1000, result
