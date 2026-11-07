# 抓取延迟修复版

尚未经过真机验证。详细改动见 README_修复说明.md。

## 下载到自己的 Windows 电脑

安装 Git 和 Python 3.8 或以上版本，打开 PowerShell 执行：

```powershell
git clone https://github.com/zczczc213472/grasp-fixed.git
cd grasp-fixed
```

## 本机离线测试：不会连接或操作机器人

```powershell
python -m unittest discover -s tests -v
```

如果 Python 命令不可用，尝试：

```powershell
py -m unittest discover -s tests -v
```

预期：`Ran 8 tests`、`OK`。离线测试只需 Python 标准库，不需要 numpy、OpenCV 或 Galbot SDK。测试使用模拟状态，不证明实际抓取成功或硬件安全。

## 文件说明

- robot_exec.py：机器人端常驻 SDK 执行器。
- tools/grasp_client.py：请求超时、夹爪轮询和稳定接触判断。
- tools/oneshot_grasp.py：一键抓取入口。
- tools/plan_grasp.py：头部观察后抓取。
- tools/drive_and_grasp.py：含底盘移动的抓取入口。
- tests/test_fixes.py：离线模拟测试。
- README_修复说明.md：部署、修改及限制。

## 真机测试另行进行

离线测试时不要运行 tools 下的抓取入口：它们会通过 SSH 连接机器人并下发实际运动，drive_and_grasp.py 还会移动底盘。真实运行需要机器人网络、SSH 权限及机器人侧 Galbot SDK、numpy、OpenCV。原运行环境为 macOS/Linux；Windows 真机连接尚未验证。

部署时同时复制 robot_exec.py 和整个 tools 文件夹，不能仅替换一个文件。此执行通路仍无碰撞规划；关节变化限制不能代替碰撞检查。通信超时只停止后续发送，不保证已下发动作立即停止。夹持阈值仍需现场标定。
