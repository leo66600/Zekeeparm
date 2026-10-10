# ZEKEEP 指擎串口舵机标定工具

本工具通过本机网页操作中凌串口舵机，可扫描和修改 ID、读取 PWM、释放扭矩及标定机械零位。

## 启动

先按[遥操作 README](../../README.md)安装独立环境及本包。设置实际 `ZKEEP_WS`，在终端执行：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/lerobot"
cd "$ZKEEP_WS/src/zekeep_teleop/src"
python -m servo_gui.server
```

浏览器打开 <http://127.0.0.1:8770>。按 `Ctrl+C` 停止服务。

无硬件演练：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/lerobot"
cd "$ZKEEP_WS/src/zekeep_teleop/src"
python -m servo_gui.server --simulate
```

## 操作注意事项

改写 ID 前必须只连接目标舵机，并按页面要求输入确认文本。ID 改写后自动设置开机释力；该舵机断电重启前不会再回包。

本工具与遥操作统一使用 `001–007` 编号；`002/003` 的目标 PWM 为 2400，其余轴为 1500。软件配置不会自动改写硬件 ID，请先确认实物编号与对应关节，再进行改号或零位标定。使用本工具前先停止占用同一串口的遥操作程序。

服务绑定 `127.0.0.1`，只供本机浏览器使用。`--simulate` 是仍保留的模拟舵机模式，
不访问串口；真实模式的改号和零位操作会写硬件，不能与模拟模式混淆。
主臂 PWM/方向标定保持现有值，不套用 ROS 六轴方向参数。

机械零位标定先等待 `PULK` 释力确认，再检查位置稳定性。`PSCK` 只发送一次，不等待标定回包；随后在约 350 ms 保存窗口内每 10 ms 补发 `PULK`，避免只发送一次释力时可能被固件保存过程忽略。保存后再次发送 `PULK` 并检查回包、回读 PWM。连续发送的回包可能混合，不能分别证明命令已执行。该流程在单独无负载的 007、`ZL-ZServo_AD_CBM V2.1.16STG` 上实测：标定前读数为 1724，标定后连续三次为 1500，操作者确认无可见转动。其他关节及负载下尚未实测，不能保证完全没有位移；执行时持续扶稳关节。没有重复写入 PSCK，不改写开机位置；本次尚未验证断电重启后的保存状态。

若 `PULK` 无回包，但实物已经可以轻松手动转动，可在本次标定弹窗勾选「我已轻转本次目标关节，确认已释力」。默认不勾选，每次打开弹窗都会清除；不要用力扳动或在仍有保持力时勾选。该选项仅允许释力回包为空时使用人工确认，异常回包仍拒绝，位置采样和标定回读检查照常执行。标定后若释力仍无回包，只验证 PWM 标定结果，`torque_restored` 为 `null`；弹窗保留提示，必须扶稳并重新检查实物，不能据此认定标定后仍已释力。该选项不会保证 PSCK 完全没有瞬间位移。

无硬件回归检查：在本包目录运行 `PYTHONPATH=src python3 -m unittest servo_gui.test_calibrate_zero`。
