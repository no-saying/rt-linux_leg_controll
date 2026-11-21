# AMP 控制协议 v0.1

本文定义遥控器/RT-Thread 与控制框架之间的 AMP 核间通信协议。传感器数据和遥控
命令只通过 AMP 进入本控制框架。USB 用于其他外设通信，不承载 AMP，也不作为控制
数据输入的备选链路。

## 方向和职责

```text
遥控器 → RTThread → AMP 发送端 → AMP 接收端 → robot_cmd → 状态机/算法
控制框架 → AMP 遥测/ACK → RTThread
```

RT-Thread 内核驱动、AMP 硬件收发、DMA、中断和硬件寄存器由板端工程负责。Global
ESO 也在 RT-Thread 侧运行；ESO 输出与同一采样周期的传感器数据一起放入 `0x12`
快照。接收框架只解码并传递观测值，不重复运行 ESO。仓库保留的 `GlobalEso` C++
实现是 RT-Thread 移植/交接参考。USB 完全位于当前控制数据路径之外。

## 线帧

所有整数小端序，浮点线值使用 IEEE-754 binary16（float16）。接收后立即转换为
内部 `float`，算法层不使用 float16 运算。CRC 使用 CRC-16/CCITT-FALSE，覆盖
`version` 到 `payload`，不覆盖两个同步字节和 CRC 本身。

| 字段 | 类型 | 字节数 |
|---|---:|---:|
| `sof0` | uint8 | 1 |
| `sof1` | uint8 | 1 |
| `version` | uint8 | 1 |
| `message_type` | uint8 | 1 |
| `flags` | uint16 | 2 |
| `payload_len` | uint16 | 2 |
| `sequence` | uint16 | 2 |
| `timestamp_ms` | uint32 | 4 |
| `payload` | bytes | N |
| `crc16` | uint16 | 2 |

同步字节固定为 `0xA5 0x5A`，当前版本为 `1`。接收端必须检查长度上限、版本、
消息类型、序号和 CRC；失败帧不得更新命令或传感器快照。

## 控制命令 `0x01`

payload 固定 16 字节：

| 偏移 | 字段 | 类型 | 含义 |
|---:|---|---:|---|
| 0 | `x_speed` | float16 | 前后速度 vx，m/s |
| 2 | `y_speed` | float16 | 横向速度 vy，m/s |
| 4 | `z_speed` | float16 | 偏航速度 wz，rad/s |
| 6 | `leg_height` | float16 | 绝对腿长目标 h，m |
| 8 | `vehicle_state` | uint32 | 车辆使能状态 |
| 12 | `motion_state` | uint32 | 车辆运动状态 |

`vehicle_state`：

```text
0 DISABLED     禁止输出扭矩
1 ENABLED      允许进入运动控制
2 ESTOP        立即进入安全停机
3 FAULT        故障保持，等待复位
```

`motion_state`：

```text
0 STOP         停车/平衡保持
1 LOCOMOTION   正常行进
2 STEP_UP      上台阶
3 JUMP         跳跃
4 RECOVERY     翻倒恢复
5 STEP_DOWN    预留
```

`leg_height` 是绝对目标，不是增量。所有 float16 必须是有限值；超出配置工作区
时由应用层拒绝或限幅，并记录故障事件。状态字高位保留，当前必须为零。

## AMP 传感器快照 `0x12`

传感器快照也是 AMP 输入，固定 payload 为 82 字节。40 个数值按下列顺序编码为
IEEE-754 binary16、小端序，最后 2 字节为 flags：

| 顺序 | 字段 | 数量 | 单位/说明 |
|---:|---|---:|---|
| 1 | `state[0..14]` | 15 | pitch, pitch-rate, roll, roll-rate, common leg swing, swing rate, mean leg length, length rate, left-right length delta, delta rate, mean wheel linear speed, differential wheel speed, yaw rate, differential swing, differential swing rate |
| 2 | `gyro[0..2]` | 3 | 安装坐标系陀螺仪，rad/s |
| 3 | `specific_force[0..2]` | 3 | 安装坐标系比力，m/s^2 |
| 4 | `joint_position[0..5]` | 6 | lf, lr, rf, rr, left wheel, right wheel，关节位置单位由板端配置固定 |
| 5 | `joint_velocity[0..5]` | 6 | 同上，单位/s |
| 6 | `body_vx, body_vy` | 2 | 机体系平面速度，m/s |
| 7 | `body_yaw` | 1 | 连续航向角，rad |
| 8 | `leg_height` | 1 | 当前腿长，m |
| 9 | `eso_disturbance[roll,pitch,yaw]` | 3 | RT-Thread Global ESO 估计的三轴集总扰动状态，单位 rad/s^2，按控制模型定义 |
| 10 | `flags` | 1 word | bit0 身体与地面平行；bit1 腿回到中位；bit2 翻倒；bit3 ESO 有效；bit4 姿态有效；bit5 IMU比力/角速度有效；bit6 15维运动学状态有效；bit7 关节位置/速度有效；bit8 轮速有效；bit9 机体平面速度有效；bit10-15必须为零 |

所有传感器值与 ESO 结果必须属于同一采样/估计时刻，并使用同一 AMP 帧序号和
时间戳发布。对应字段无效时必须清除其有效位；数值为零不代表有效。bit3 为 0 时，
接收端必须把 ESO 结果视为无效；NMPC 还要求 bit4、bit6、bit8、bit9 均有效。RT-Thread
侧在首次有效观测前应清除 bit3。

AMP 帧头时间戳是发送端诊断时间，不用于本地超时比较。接收 BSP 把本地单调时钟
的接收时刻传给 `ControlRuntime`，命令与传感器使用独立序号，并都用本地接收时钟
进行超时判断。序号 16 位回绕按半区间新旧关系判断。

## ACK `0x02`

payload 固定 12 字节：`uint16 accepted_sequence`、`uint8 result`、`uint8 reason`、
`uint32 active_vehicle_state`、`uint32 active_motion_state`。ACK 只表示协议接收和
边界校验结果，不表示电机已经达到目标。

## 遥测 `0x10`

第一版最小遥测 payload 固定为：时间戳、当前状态字、活动模式、故障码、四个
float16 目标回显、六路实际扭矩 float16。扩展字段必须增加版本或消息类型，不能
改变已发布 payload 的字段顺序。

## 时序和失联

- 命令建议 100 Hz 发送；
- 连续超过 50 ms 没有有效命令，应用层进入 `DISABLED` 或安全保持；
- 传感器快照也以本地接收单调时钟检查新鲜度，连续超过 50 ms 未更新时禁止控制输出；
- 序号允许回绕，但旧帧、重复帧和乱序帧不得覆盖新目标；
- `ESTOP` 和 CRC/长度错误处理优先级高于运动状态；
- AMP 解析不得调用 malloc、free、异常或阻塞 IO。

## float16 限制

float16 只用于线传输以节省带宽。姿态、速度、腿长、力矩和 ESO 观测值在接收后
转换为 `float` 或 `double`。协议实现必须明确处理 subnormal、Inf、NaN 和溢出；
NaN/Inf 直接拒绝，有限溢出按协议范围拒绝，不静默变成最大值。

## 安全快照 `0x13`

payload 固定 12 字节：pitch/roll（各 float16，rad）、三轴比力（各 float16，m/s^2）、
flags（uint16）。flags bit0 表示姿态有效，bit1 表示翻倒状态，bit2 表示身体与地面
平行，bit3 表示比力有效。RT-Thread 仅在 ACCL 与 EULR 接收时间相差不超过 10 ms 且
都未过期时置 bit3；否则比力无效。Linux 将翻倒标志锁存到恢复流程，不会因后续单帧
姿态恢复而清除恢复请求。

板级 CAN 路由：CAN1 接两个 M3508 轮毂和 ATOM IMU，CAN2 接四路关节电机。六路电机
数据中的最后两路是轮毂，不能按六路关节解释。当前缺少完整腿部运动学和机体速度来源，
不能据此构造虚假的完整 `0x12` NMPC 状态。
