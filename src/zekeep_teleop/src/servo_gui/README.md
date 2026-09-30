# ZEKEEP 指擎串口舵机标定工具

本工具通过本机网页操作中凌串口舵机，可扫描和修改 ID、读取 PWM、释放扭矩及标定机械零位。

## 启动

先按[遥操作 README](../../README.md)安装独立环境及本包。设置实际 `ZKEEP_WS`，在终端执行：

```bash
conda activate lerobot
cd "$ZKEEP_WS/src/zekeep_teleop/src"
python -m servo_gui.server
```

浏览器打开 <http://127.0.0.1:8770>。按 `Ctrl+C` 停止服务。

无硬件演练：

```bash
conda activate lerobot
cd "$ZKEEP_WS/src/zekeep_teleop/src"
python -m servo_gui.server --simulate
```

## 操作注意事项

改写 ID 前必须只连接目标舵机，并按页面要求输入确认文本。ID 改写后自动设置开机释力；该舵机断电重启前不会再回包。

本工具与遥操作统一使用 `001–007` 编号；`002/003` 的目标 PWM 为 2400，其余轴为 1500。软件配置不会自动改写硬件 ID，请先确认实物编号与对应关节，再进行改号或零位标定。使用本工具前先停止占用同一串口的遥操作程序。

服务绑定 `127.0.0.1`，只供本机浏览器使用。`--simulate` 是仍保留的模拟舵机模式，
不访问串口；真实模式的改号和零位操作会写硬件，不能与模拟模式混淆。
主臂 PWM/方向标定保持现有值，不套用 ROS 六轴方向参数。
