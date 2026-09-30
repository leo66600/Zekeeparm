# zekeep_teleop：zk_xm115 从臂遥操作

本项目通过 LeRobot 将七轴舵机主臂的动作实时映射到 `zk_xm115` 七轴从臂。

本目录随 `Zekeeparm_ws` 统一维护，保留独立的 Python/LeRobot 环境。
`COLCON_IGNORE` 将其排除在 ROS 构建之外；无需启动 ROS 或 MoveIt。
不要将本项目依赖安装进 `.venv-ros`，两者使用不同版本的 `motorbridge`。

## 硬件与软件基线

串口舵机网页标定工具已同步至 [src/servo_gui](src/servo_gui/README.md)，启动方法见该目录说明。工具与遥操作统一使用 `001–007` 编号，按关节顺序对应从臂 CAN ID `001–007`。软件配置修改不会自动改写舵机硬件 ID；运行前必须确认实物编号一致。

- 操作系统：Ubuntu 22.04 或 Ubuntu 24.04 x86_64
- Python：`3.12`
- LeRobot：`0.4.4`
- 主臂：七轴舵机臂，串口波特率 `115200`
- 从臂：`zk_xm115`，Damiao USB-CAN
- 控制频率：`50 Hz`
- 设备端口：以目标计算机实际识别结果为准
- 已验证主臂方向：`[1, 1, 1, -1, -1, 1, 1]`

本包及采集器保持原有角度制、主从映射和启动零位标定。ROS/视觉/网页的
J1 ±2.58 rad、J2/J3 0–3.7 rad 及六轴 `-1` 方向不覆盖本包配置。

当前关节映射如下。角度为已经完成实机验证的程序动作范围，不要用未经上机验证的标称角度覆盖。

| 主臂舵机 ID | zk_xm115 关节 | 从臂 CAN ID | 程序动作范围 |
|---:|---|---:|---:|
| 001 | shoulder_pan | 001 | -149.54°～149.54° |
| 002 | shoulder_lift | 002 | -211.99°～0° |
| 003 | elbow_flex | 003 | -211.99°～0° |
| 004 | wrist_flex | 004 | -89.95°～89.95° |
| 005 | wrist_yaw | 005 | -89.95°～89.95° |
| 006 | wrist_roll | 006 | -89.95°～89.95° |
| 007 | gripper | 007 | 0°～85.94°（从臂侧反向） |

主臂舵机 002/003 对应从臂 CAN 002/003。沿用这两个物理关节已验证的 `+1` 方向；原主臂 ID 002 的肘关节比例系数现在属于 ID 003。

## 1. 安装系统依赖

更新软件源并安装基础工具：

```bash
sudo apt update
sudo apt install -y git curl ffmpeg build-essential
```

## 2. 安装 Miniforge

如果计算机已经可以使用 Conda，可以跳过本节。x86_64 计算机执行：

```bash
curl -L -o /tmp/Miniforge3-Linux-x86_64.sh \
  https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash /tmp/Miniforge3-Linux-x86_64.sh -b -p "$HOME/miniforge3"
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda init bash
```

完成后重新打开终端。ARM64 计算机应改用 `Miniforge3-Linux-aarch64.sh`。

## 3. 获取项目并创建虚拟环境

```bash
# 已获取总代码包后，设置实际工作区路径。
export ZKEEP_WS="/你的实际工作区路径"
cd "$ZKEEP_WS/src/zekeep_teleop"

conda create -n lerobot python=3.12 -y
conda activate lerobot
python -m pip install --upgrade pip
python -m pip install -e .
```

安装命令会安装以下主要依赖：

- `lerobot==0.4.4`
- `lerobot-robot-seeed-b601==1.0.0`
- `motorbridge==0.4.9`
- `pyserial==3.5`

确认安装结果：

```bash
python -m pip show lerobot lerobot-teleoperator-zhongling \
  lerobot-robot-seeed-b601 motorbridge pyserial
```

## 4. 配置串口权限

将当前用户加入 `dialout` 用户组：

```bash
sudo usermod -aG dialout "$USER"
```

执行后注销并重新登录，使用户组权限生效。不要长期使用 `chmod 777` 代替串口权限配置。

## 5. 连接并识别设备

先连接主臂，再连接 zk_xm115 的 Damiao USB-CAN 适配器，然后执行：

```bash
python -m serial.tools.list_ports -v
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
```

配置文件位于 `configs/zhongling_b601_teleop.yaml`，其中端口仅为当前设备示例：

```yaml
robot:
  port: /dev/ttyACM1
  id: zk_xm115
teleop:
  port: /dev/ttyUSB0
```

不同计算机上的设备编号可能不同。根据识别结果修改两个 `port`，长期部署建议使用 `/dev/serial/by-id/...` 稳定路径。

检查串口是否被其他程序占用：

```bash
fuser -v /dev/ttyUSB0 /dev/ttyACM1
```

遥操作运行期间不要同时启动串口监视器、旧遥操作程序或其他访问同一设备的进程。

## 6. 启动前检查主臂

此时先不要使能 zk_xm115。从臂周围保持无人员、无障碍物，并确认急停开关有效。

把主臂摆到机械零位，使用只读工具确认舵机 001～007 都能通信：

```bash
python tools/servo_probe.py --port /dev/ttyUSB0 --start 1 --end 7
```

只读监控单个舵机的 PWM 和映射角度：

```bash
zhongling-tune \
  --config-path=configs/zhongling_b601_teleop.yaml \
  --servo-id=3 \
  --monitor
```

启动时，主臂舵机 002/003 必须接近 PWM 2400，容差为 ±60。程序会把完整启动姿态作为本次会话零位。如果出现 `is not at startup zero`，应停止并重新检查主臂机械零位。

舵机中点和上电位置保存在舵机内部，换计算机不需要重新写入 PSCK 或 PCSD。

## 7. 启动遥操作

首次在新计算机运行时，应支撑 zk_xm115，从小幅单关节动作开始检查方向和限位，再逐步扩大动作范围。

```bash
conda activate lerobot
cd "$ZKEEP_WS/src/zekeep_teleop"
zhongling-teleoperate --config_path=configs/zhongling_b601_teleop.yaml
```

必须使用本项目提供的 `zhongling-teleoperate`。该入口启用后台主臂读取，并在不显示相机时跳过从臂 observation 轮询，以保持低延迟控制。

正常日志应包含：

```text
ZhonglingLeader captured startup pose as session zero
ZhonglingLeader connected on /dev/ttyUSB0 @ 115200
SeeedB601DMFollower connected
Teleop: 49.x Hz ... target 50 Hz
```

按 `Ctrl+C` 停止。程序会先断开 zk_xm115 并释放从臂力矩，再关闭主臂串口。

## 8. 常见问题

### 串口权限不足

如果出现 `Permission denied`，确认当前用户属于 `dialout`：

```bash
groups
```

加入用户组后必须重新登录。

### 找不到默认端口

重新运行 `python -m serial.tools.list_ports -v`，然后按实际设备路径修改 YAML。不要假设另一台计算机仍然使用 `/dev/ttyUSB0` 和 `/dev/ttyACM1`。

### 主臂舵机没有完整响应

出现 `No valid PRAD response` 时，停止遥操作并检查主臂电源、串口、波特率、总线接线和舵机 ID。001～007 未全部响应前不要使能从臂。

### 从臂不运动或方向错误

确认主臂方向没有被旧配置覆盖：

```yaml
teleop:
  joint_directions: [1, 1, 1, -1, -1, 1, 1]
```

核对当前环境加载的是本工作区的插件及配置；不要通过改写舵机零位来掩盖配置不一致。

### 控制频率明显低于 50 Hz

确认启动命令为 `zhongling-teleoperate`，串口没有被其他进程占用，并保持：

```yaml
read_robot_observation: false
teleop:
  use_background_read: true
```

## 9. 配置修改与验证

端口修改后，先按“连接并识别设备”和“启动前检查主臂”确认通信。

如果修改了关节方向、PWM 映射、关节限位、速度、控制频率或依赖版本，必须同步更新配置和本文档，并在急停就绪、从臂受支撑的条件下重新进行低速单轴验证。

## 相关模块

- [工作区入口](../../README.md)：ROS 与视觉环境、模型和接口。
- [舵机网页标定](src/servo_gui/README.md)：仅操作主臂串口舵机。
