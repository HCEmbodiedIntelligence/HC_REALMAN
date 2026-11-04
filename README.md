# HC_REALMAN

RealMan ECO65-B 双臂 + 两只智元灵犀 X1 OmniPicker 的 ROS 2 Humble 底层驱动。
接口风格与 `HC_X1` 一致：上层向 `/io_teleop/joint_cmd` 下发双臂关节目标，
驱动发布 `/io_teleop/joint_states`、`/io_teleop/hardware_ready`，夹爪使用独立的
左右命令和反馈话题。

夹爪通过 ECO65-B 原厂末端两线 RS485 接口连接。RealMan 控制器配置为工具端
Modbus-RTU 主站；每侧机械臂独立控制自己的夹爪，不需要 USB-RS485 或 CAN 网关。
OmniPicker 必须先用官方 REF-CLI 切到 RS485 + Modbus 模式并保存重启。

## 安全默认值

- 默认 `arms_control: false`、`hands_control: false`；不发运动命令，也不对夹爪上电；
- 默认不自动回零，避免首次部署意外运动；
- 双臂各自连接、各自 watchdog，任何一侧反馈过期都会使总 ready 变为 false；
- 关节命令按名字匹配、拒绝 NaN/Inf/重复关节、限位外命令和过期命令；
- OmniPicker 上电会自动闭合找零，周围必须无遮挡；禁止带电插拔 6Pin 线。

## 配置

编辑 `config/hardware.yaml`：

- `left_arm.ip` / `right_arm.ip`：两台 ECO65-B 的控制器地址；
- `left_arm.joints` / `right_arm.joints`：必须与上层模型完全一致；
- `hands.baudrate`：必须与两只夹爪一致，支持 115200 或 460800；
- `hands.*.device_id`：Modbus 从站地址。两侧是独立 RS485 总线，可以相同。

夹爪固件需不低于 3.3.5。用 USB-C 接 REF-CLI，把每只夹爪设为：
`rs485_mode=1`、`modbus_mode=True`、所选波特率，保存配置并重新上电。
末端供电由驱动配置为 24V；首次启动前仍应核对夹爪铭牌和线序。

## 构建与启动

```bash
cd /home/maple/test2/HC_REALMAN
/usr/bin/python3 -m pip install -r requirements.txt
./build.sh
./start.sh                       # 默认全只读
./start.sh arms_control:=true    # 只开启双臂控制
./start.sh arms_control:=true hands_control:=true
```

项目固定使用 RealMan Python API2 1.1.5。厂商运行库通过 PyPI 安装，不纳入本
仓库；官方包会按 Linux x86-64/aarch64 自动选择运行库。控制器固件需与 API2
兼容，升级前应先在只读模式验证。

控制前必须先在只读模式确认双臂 IP、12 个关节方向/顺序、夹爪 ID 和反馈正常。
`check.sh` 用于快速检查话题和 ready 状态。

## ROS 2 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/action/joint_states` | `sensor_msgs/JointState` | 标准 12 轴目标，rad，内置限速/限加速度 |
| 订阅 | `/io_teleop/joint_cmd` | `sensor_msgs/JointState` | 兼容私有目标；与标准话题互斥仲裁 |
| 发布 | `/io_teleop/joint_states` | `sensor_msgs/JointState` | 12 轴反馈，rad |
| 发布 | `/observation/joint_states` | `sensor_msgs/JointState` | 标准 12 轴反馈，rad |
| 发布 | `/io_teleop/hardware_ready` | `std_msgs/Bool` | transient-local |
| 发布 | `/status/hardware/ready` | `std_msgs/Bool` | 标准硬件就绪状态 |
| 服务 | `/io_teleop/reset_robot` | `std_srvs/Trigger` | 清除双臂系统/关节错误，不执行运动 |
| 订阅 | `/action/left_hand/joint_states` | `sensor_msgs/JointState` | 左夹爪，position 归一化 0=闭合、1=全开 |
| 订阅 | `/action/right_hand/joint_states` | `sensor_msgs/JointState` | 右夹爪，同上 |
| 发布 | `/observation/hands/joint_states` | `sensor_msgs/JointState` | 两夹爪实际归一化位置 |
| 发布 | `/status/hands/raw` | `std_msgs/UInt16MultiArray` | left/right 的 fault,state,pos,vel,force |

夹爪只在 `hands_control:=true` 时上电和建立 Modbus 通信，因为 OmniPicker 的
上电找零会产生一次真实闭合动作。双臂目标在驱动内从最新反馈开始生成轨迹，默认
限制为 45°/s、120°/s²；命令断流、ready 变 false 或节点退出时发送一次当前位置
保持命令并清空目标。

RealMan 官方 API2 的 `movej_canfd` 名称指机械臂内部关节角度透传；夹爪通信走
`rm_*modbus_rtu*` 工具端主站接口，两条链路彼此独立。
