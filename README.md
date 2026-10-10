# Zekeeparm

六轴机械臂独立运行包，覆盖真机控制、MoveIt、视觉抓取、拖动示教和遥操作。

**目标平台：** Ubuntu 22.04 x86_64 · ROS 2 Humble · Python 3.10

[项目组成](#项目组成) · [一键安装与环境配置](#一键安装与环境配置) · [NVIDIA 驱动与 CUDA 12.8](#nvidia-驱动与-cuda-128) · [运行入口](#运行入口) · [标定边界](#标定边界) · [许可与安全提示](#许可与安全提示)

---

## 项目组成

| 模块 | 功能 |
| --- | --- |
| `zekeep_bringup` | 模型、硬件参数、真机、RViz 和网页启动 |
| `zekeepcontroller` | 硬件抽象、状态机、ROS 服务和轨迹 Action |
| `zekeep_msgs` | 自定义消息、服务和 Action |
| `zekeep_moveit_config` | MoveIt 运动规划配置 |
| `zekeep_teach` | 重力补偿拖动示教、网页任务和轨迹复现 |
| `zekeep_shadow` | 网页 RGB-D 检测及 YOLO/GraspNet 抓取后端 |
| `zekeep_llm` | 网页 LLM/MCP 计划、人工确认执行和取消 |
| `zekeep_joystick` | 六轴关节及末端手柄控制 |
| `zekeep_grasp` | 相机、当前标定、检测和抓取 |
| `zekeep_teleop` | 独立 LeRobot 主从遥操作 |
| `zekeeparm_SDK` | 机械臂底层 SDK，仅保留 DM 硬件配置 |

> **模块说明**：
> 1. GraspNet 源码受开源协议限制，不随本仓库直接分发，由安装者接受许可后通过脚本获取。
> 2. 发布源码不包含 MuJoCo、具身学习与示范数据采集模块。
> 3. 模型权重、编译产物、日志及运行缓存由本地安装或运行生成，不属于发布源码清单。

---

## 一键安装与环境配置

> [!IMPORTANT]
> 安装过程需要连接外网，脚本会调用 `sudo apt`。NVIDIA 显卡驱动与 CUDA Toolkit 12.8 需单独配置（见[下方章节](#nvidia-驱动与-cuda-128)）。安装脚本不会连接硬件或使能电机。

项目采用三套环境隔离策略：

- **ROS 驱动**：使用独立系统虚拟环境 `.venv-ros`。
- **视觉抓取**：使用 Conda 环境 `rebotarm`（Python 3.10，NumPy 1.26.4）。
- **主从遥操作**：使用 Conda 环境 `lerobot`（Python 3.12）。

### 1. 检查本机 Conda 状态

在终端执行以下命令，确认本机是否已安装 Conda 及其根目录：

```bash
conda --version
conda info --base
conda env list
```

*注：`conda --version` 仅输出版本号，不会标注是 Miniconda 还是 Miniforge。可通过 `conda info --base` 查看根目录判断发行版类型：*

| 默认根目录路径 | 对应发行版 | 本包对应处理方式 |
| --- | --- | --- |
| `$HOME/miniforge3` | Miniforge | 默认路径，无需手动设置变量 |
| `$HOME/miniconda3` | Miniconda | 将 `ZKEEP_MINIFORGE_DIR` 设为该目录 |
| `$HOME/anaconda3` | Anaconda | 将 `ZKEEP_MINIFORGE_DIR` 设为该目录 |

**若提示 `conda: command not found`**，可检查默认路径是否存在可执行程序：

```bash
ls -l "$HOME/miniforge3/bin/conda" \
      "$HOME/miniconda3/bin/conda" \
      "$HOME/anaconda3/bin/conda" 2>/dev/null
```

若文件存在，手动加载对应的初始化脚本即可：

```bash
# 以 Miniconda 为例
source "$HOME/miniconda3/etc/profile.d/conda.sh"
```

若上述路径均不存在，按下文“未安装 Conda”流程处理。

---

### 2. Conda 根路径配置

项目通过环境变量 `ZKEEP_MINIFORGE_DIR` 指定 Conda 根目录（虽含 miniforge 字样，但**完全兼容 Miniconda/Miniforge/Anaconda**）。

- **已有 Miniconda / Anaconda 时**：

无需重复安装 Miniforge，直接将变量指向已有 Conda 根路径：

```bash
# 方式 A：自动获取当前激活的 Conda 根目录
export ZKEEP_MINIFORGE_DIR="$(conda info --base)"

# 方式 B：手动指定绝对路径（以默认 Miniconda 为例）
export ZKEEP_MINIFORGE_DIR="$HOME/miniconda3"
```

安装脚本将在该目录的 `envs/` 下创建 `rebotarm` 与 `lerobot` 环境。

- **未安装 Conda 时（推荐全新电脑）**：

无需手动下载安装，**直接执行步骤 3 的 `bash setup.sh`。**脚本将自动下载并校验官方 `Miniforge3-Linux-x86_64.sh`，默认安装至 `$HOME/miniforge3`。如需自定义安装目录，在运行脚本前设置：

```bash
export ZKEEP_MINIFORGE_DIR="/你的自定义路径/miniforge3"
```

> **建议**：将导出的 `export ZKEEP_MINIFORGE_DIR=...` 写入 `~/.bashrc`，避免在新终端中失效。

---

### 3. 克隆仓库并执行安装

```bash
git clone https://github.com/Leocia/Zekeeparm.git "$HOME/Desktop/Zekeeparm"
cd "$HOME/Desktop/Zekeeparm"
bash setup.sh
```

脚本将自动执行以下操作：

1. 安装 ROS 2 Humble 相关系统依赖与工具；
2. 构建 ROS 驱动虚拟环境 `.venv-ros` 并安装底层 SDK；
3. 创建视觉 Conda 环境（`rebotarm`）与遥操作 Conda 环境（`lerobot`）；
4. 下载视觉模型基础权重。

**GraspNet 下载与编译交互**：

运行脚本期间会提示是否获取 GraspNet。确认用于个人/机构内部非商业研究并输入 `YES` 后，脚本将自动拉取源码、下载 `checkpoint-rs.tar` 权重，并在当前 GPU 上编译 `pointnet2` 和 `knn` CUDA 扩展。若未输入 `YES` 或环境暂无 CUDA 12.8，脚本会跳过 GraspNet 原生编译，基础 ROS 驱动与仿真不受影响。

---

### 4. 验证视觉环境

安装完成后，验证视觉环境及依赖是否正确就绪：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
python -c 'import sys; print("Python 路径:", sys.executable); print("Python 版本:", sys.version)'
python -c 'import numpy, cv2, torch, ultralytics, pyorbbecsdk; print("✓ 视觉基本依赖导入成功")'
```

*注：若视觉环境使用了非标准命名或独立自定义路径，可通过环境变量指定解释器（优先级高于 `ZKEEP_MINIFORGE_DIR`）：*

```bash
export ZKEEP_VISION_PYTHON="/实际视觉环境路径/bin/python"
# 更换回默认配置时执行：unset ZKEEP_VISION_PYTHON
```

---

## NVIDIA 驱动与 CUDA 12.8

使用 YOLO/GraspNet 视觉抓取原生扩展需要配置 NVIDIA 显卡驱动与 CUDA 12.8。

### 1. 显卡驱动安装

```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
sudo reboot
```

重启后执行 `nvidia-smi`，确认驱动正常工作。

### 2. CUDA Toolkit 12.8 安装

采用官方独立 Toolkit 安装（避免使用 `cuda` 元包意外覆盖已有驱动）：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-8
```

### 3. 环境变量设置与编译

在当前终端设置 CUDA 路径，并重新执行安装脚本以编译扩展：

```bash
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
cd "$HOME/Desktop/Zekeeparm"
bash setup.sh
```

---

## 运行入口

### 1. ROS 2、RViz、网页、手柄与示教

每个 ROS 终端需加载工作区环境：

```bash
cd "$HOME/Desktop/Zekeeparm"
source install/setup.bash
```

**自动加载配置（可选）**：若希望新终端自动加载环境，在 `~/.bashrc` 末尾添加一次：

```bash
if [ -f /home/zekeep/Desktop/Zekeeparm/install/setup.bash ]; then
  source /home/zekeep/Desktop/Zekeeparm/install/setup.bash
fi
```

保存后执行 `source ~/.bashrc`，运行 `ros2 pkg prefix zekeep_bringup` 验证包路径。若迁移路径需同步修改。

- **无硬件仿真（MoveIt + RViz 界面）：**

```bash
ros2 launch zekeep_moveit_config demo.launch.py
```

- **网页交互界面：**

```bash
ros2 launch zekeep_bringup web.launch.py
```

默认访问地址：`http://127.0.0.1:3001`（包含 TCP 预览、实机示教、LLM/MCP 自然语言交互）。

- **网页视觉抓取统一启动：**

```bash
"$HOME/Desktop/Zekeeparm/tools/start_web_sim.sh"
```

*启动时不自动使能电机；AI 规划运动需在网页中点击确认执行。仅观察预览可增加 `--preview` 参数。*

---

### 2. 独立视觉抓取

运行视觉抓取节点需独占串口，不能与 ROS 驱动同时运行：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
cd "$HOME/Desktop/Zekeeparm/src/zekeep_grasp"
```

---

### 3. 主从遥操作

启动基于 LeRobot 的双臂遥操作：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/lerobot"
cd "$HOME/Desktop/Zekeeparm/src/zekeep_teleop"
```

> **控制与机械参数**：
> - 夹爪行程采用 `1.45 rad = 70 mm` 线性映射；电机运动限制为 `0–1.35 rad`（最大允许开口约 `65.17 mm`）。
> - ROS 与底层 SDK URDF 模型一致，保留 `official_tcp` 兼容坐标系。
>

---

## 标定边界

仓库随附通用的相机内参 `intrinsics.npz` 与手眼标定 `hand_eye.npz`，但不随 Git 分发具体的设备硬件绑定记录（`identity.local.json`）。

- **设备变动、重新安装或场景位移**：必须重新进行内参及手眼外参标定。
- **环境、机架及相机完全未变**：沿用旧标定前，必须在视觉环境中核验相机硬件身份：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
cd "$HOME/Desktop/Zekeeparm"
python src/zekeep_grasp/scripts/confirm_calibration.py \
  --config src/zekeep_grasp/config/default.yaml --confirm-same-installation
```

*执行前请在 `src/zekeep_grasp/config/default.yaml` 中如实填入设备的 `camera.serial`、`robot.id` 与 `calibration.installation_id`。该命令仅打开相机进行匹配核验，不使能电机。*

---

## 许可与安全提示

> [!WARNING]
> 1. **学术研究限制**：本工作区仅供个人或同机构内部进行非商业科研研究，严禁用于任何商业用途或向第三方转让分发。GraspNet 源码与模型权重受其官方协议约束。
> 2. **硬件操作安全**：安装脚本及全部启动命令默认均**不会自动使能电机**。任何真机运动前，操作人员必须确认急停开关处于可触发状态、机械臂运动包络内无障碍物与人员。操作者须对实体设备的运行及由此产生的后果负全部责任。
>
