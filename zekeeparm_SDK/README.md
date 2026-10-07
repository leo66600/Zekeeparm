# zekeeparm_SDK

Zekeeparm 机械臂 Python 控制 SDK，包含电机分组控制、正逆运动学、动力学、末端控制和轨迹规划。

从工作区根目录安装：

```bash
python -m pip install --no-deps -e zekeeparm_SDK
```

唯一默认硬件配置为 `config/rebotarm_dm.yaml`（达妙电机）。保留原有电机参数及模型设置。
ROS 控制器仍可传入已合并的运行时硬件配置。

```python
from zekeeparm_SDK.actuator import RebotArm
from zekeeparm_SDK.controllers import RebotArmEndPose
```

包名由 `reBotArm_control_py` 改为 `zekeeparm_SDK`，控制类名保持不变。
完整安装说明见工作区的 `docs/SDK_INSTALL.md`。
