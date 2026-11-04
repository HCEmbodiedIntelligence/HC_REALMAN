# 部署与接线

## 网络

主控需同时访问两台 ECO65-B 的 TCP 8080 端口。默认示例为左臂
`192.168.1.18`、右臂 `192.168.1.19`，必须按现场地址修改。两台控制器 IP
不可相同。

## 末端 RS485

每只 OmniPicker 的两根通信线分别接对应 ECO65-B 末端的 RS485 A/B；电源和
GND 也接对应末端。禁止带电插拔。若一直读超时，先断电，核对 A/B 是否接反、
设备地址、波特率及夹爪是否已切到 Modbus 模式。

夹爪配置要求：固件 >= 3.3.5，`rs485_mode=1`，`modbus_mode=True`，波特率
与 `hardware.yaml` 相同，保存后重新上电。驱动使用工具端 Modbus 主站
（RealMan API `type=1`）：写起始寄存器 10、长度 6；读起始寄存器 20、长度 5。

## 首次上电顺序

1. 机械空间清空，实体急停可触达。
2. 保持 `arms_control=false`、`hands_control=false` 启动，确认 12 个关节反馈。
3. 清空夹爪周围空间，再用 `hands_control:=true` 单独确认夹爪 Modbus 反馈。
4. 分别单侧、小幅度验证机械臂运动，再开放双侧控制。
5. 上层命令断流超过 250 ms 后驱动停止继续下发新目标。

OmniPicker 上电会自动闭合找零，这是设备固件行为，不受 ROS 控制开关阻止。

## 仿真与遥操系统配合 (HC-teleop-robotic)

本驱动与 `HC-teleop-robotic` 的 `HC_REALMAN` 适配包关节与话题命名完全对齐：
- 适配包目录：`adapters/robots/HC_REALMAN/`（包含完整 URDF、STL 网格、Kinematics 与 VR 遥操配置）
- 受控关节命名：`left_joint1` ~ `left_joint6`，`right_joint1` ~ `right_joint6`
- 仿真运行：进入 `~/test2/HC-teleop-robotic`，在 Web 仪表盘切换到 `HC_REALMAN` 或执行 `./run.sh sim` 即可开启 PyBullet 与网页三维仿真
- 真机联调：在 `HC-teleop-robotic` 执行 `./run.sh teleop`，并启动本项目 `./start.sh arms_control:=true hands_control:=true`

