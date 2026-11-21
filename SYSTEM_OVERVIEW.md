# 串联腿轮式机器人控制系统总览

本文汇总 `LINUXCODE` 控制框架与 `RTTHREADCODE` RK3568 CPU3 固件的职责、接口、数据流、当前实现状态和联调顺序。结论以当前代码和工程文档为准；第三方 acados、HPIPM、BLASFEO 源码不在本文范围内。

## 1. 系统组成

系统由两个独立工程构成：

| 工程 | 运行位置 | 主要职责 |
|---|---|---|
| `RTTHREADCODE` | RK3568 CPU3 / RT-Thread | 板级初始化、IMU/CAN/电机外设、AMP IPC、命令接收与安全门控、电机服务，以及当前已运行的 Global ESO |
| `LINUXCODE` | Linux 控制框架/主机验证环境 | AMP 帧解析与应用状态机、串联腿轴动力学模块、三轴 NMPC 接口及 acados dense-QP/HPIPM 求解路径 |

两者通过 AMP 共享内存/mailbox 通信，不是一个可直接整体编译的工程。RT-Thread 管硬件与电机输出；Linux 框架定义协议边界和控制应用算法。USB 不属于该控制输入链路。

## 2. 端到端数据流

目标运行路径如下：

```text
RT-Thread: AtomIMU / 编码器 / 电机反馈
    -> RT-Thread Global ESO
    -> 同时刻的传感器与 ESO 状态组帧为 AMP 0x12
    -> Linux AMP 接收、校验和新鲜度/安全检查
    -> ControlRuntime / ControlApplication
    -> 三轴 decoupled NMPC (roll / pitch / yaw-wheel)
    -> 六路扭矩结果
    -> 板端电机写入
```

命令方向相反：

```text
Linux -> AMP 0x01 COMMAND_SETPOINT -> RT-Thread 校验、序号检查、状态更新
      <- AMP 0x02 COMMAND_ACK      <- 仅确认协议/边界处理结果，不代表电机到位
```

### 当前实际连通状态

- RT-Thread 已从 AtomIMU 读取欧拉角/陀螺仪新样本并更新 Global ESO。IMU 数据不可用、观测超时或 ESO 更新失败时会复位 ESO 并清除有效状态。
- RT-Thread AMP IPC 接收线程已逐字节解析 `0x01`，完成帧校验后返回 `0x02`。应用有 50 ms 有效命令超时，超时回到 `DISABLED/STOP`；`MotorApplicationUpdate()` 将非 `ENABLED` 状态映射到电机输出禁止门控。
- RT-Thread 当前把 ESO 值用于诊断输出，但没有构造并发布协议定义的二进制 `0x12` 快照，也没有把完整机器人状态、关节状态与 ESO 同步打包。因此 Linux 所需的 NMPC 输入链尚未由这两个工程实际闭合。
- Linux 文档和框架代码定义/实现了传感器快照接收边界与 `ThreeNmpcControlAlgorithm`，但板端的接收 transport、电机 writer、线程后端，以及经实测确认的 NMPC 状态/参考映射仍需集成。
- 所以“求解器测试通过”不等于整车闭环通过。目前记录的 CAD/RK4 闭环复测完成了 QP 求解，但控制稳定性验收未通过；目标板实时性和整车行为仍未验收。

## 3. AMP 接口概要

协议线帧以 `A5 5A` 为同步字节，整数小端序，CRC 为 CRC-16/CCITT-FALSE；float16 只用于传输，接收后转为浮点数并拒绝 NaN/Inf 等无效输入。命令和传感器快照使用独立的 16 位序号；时间戳用于发送端诊断，超时判断使用接收端单调时钟。

| 类型 | 方向 | 用途与关键约束 |
|---|---|---|
| `0x01 COMMAND_SETPOINT` | Linux -> RT-Thread | 固定 16 字节 payload；检查版本、长度、CRC、序号、有限值和状态组合后才更新目标 |
| `0x02 COMMAND_ACK` | RT-Thread -> Linux | 固定 12 字节；报告接受/拒绝及活动状态，不代表物理执行成功 |
| `0x12 SENSOR_SNAPSHOT` | RT-Thread -> Linux | 固定 82 字节：40 个 binary16 数值及 16 位 flags；包括 15 维状态、IMU、六路关节位置/速度、机体速度/航向/腿高、三轴 ESO 扰动 |

`0x12` 的 flags.bit3 表示 ESO 有效；传感器与 ESO 必须来自同一采样/估计时刻并共享同一帧序号和时间戳。bit3 未置位时，Linux 不得将扰动值作为有效 ESO 输入，控制输出应保持无效/走安全策略。

传感器 payload 中六路关节次序为 `lf, lr, rf, rr, left wheel, right wheel`。NMPC 内部扭矩输出注释使用 `LF, LR, left wheel, RF, RR, right wheel`；板级 mapper 与 motor writer 必须显式完成顺序映射、单位/坐标变换和符号确认，不能假定两者天然相同。

协议完整字段定义和校验要求见 [AMP_PROTOCOL.md](LINUXCODE/doc/amp/AMP_PROTOCOL.md)；RT-Thread 的帧常量见 [amp_protocol.h](RTTHREADCODE/bsp/amp/amp_protocol.h)。

## 4. RT-Thread 固件

工程面向 RK3568 CPU3 AMP，RT-Thread 内核及组件从 SDK 外部 `rtos/` 目录提供。固件保留既有板端地址窗口：CPU3 镜像 `0x03000000` / `0x00200000`，AMP 共享内存 `0x07800000`；变更链接地址时必须同步 Linux DTS、AMP 驱动和 FIT 配置。

主要模块：

- `application/main.c`：初始化 AtomIMU、Global ESO、AMP 电机应用与服务，并创建外设、IPC 接收、心跳等线程。
- `application/amp_application.c`：固定缓冲的 `0x01` 流式解析、ACK、序号和命令状态/超时管理。
- `modules/algorithm/global_eso.c`：roll/pitch/yaw 三轴 Global ESO；默认带宽 `35/35/25`、扰动限幅 `30/30/20 rad/s^2`、`alpha=0.5`、`delta=0.01`。
- `modules/motor/`、`bsp/can/`、`modules/atom_imu/`：电机、CAN 和 AtomIMU 驱动/服务。
- `bsp/amp/`：共享内存 IPC 与协议定义。

当前 ESO 从 ATOM 的 pitch/roll/yaw 顺序映射到 roll/pitch/yaw；名义角加速度暂设为零，因为当前 RT 应用没有对应刚体模型项。当前 ESO 更新由约 5 ms 的 `periph` 线程驱动，尚未达到交接文档提出的 2 ms ESO 周期要求。M3508 输出轴反馈换算和 C620 电流/扭矩换算的现有约定见 [RTTHREADCODE README](RTTHREADCODE/README.md)。

## 5. Linux 控制框架与算法

Linux 框架采用固定数据结构、静态队列和有界控制路径，目标是避免控制周期动态内存分配和阻塞 IO。`ControlRuntime` 处理命令/传感器帧新旧关系、超时、安全状态及控制调用；`ControlApplication` 串接感知接口、可选 green-wave 规划和所选控制器。

当前算法主线是 RT-Thread Global ESO + 三块解耦 NMPC：roll、pitch、yaw-wheel 各有常驻上下文，ESO 扰动作为预测中的加速度项，不在 Linux 重跑 ESO，也不在求解后额外叠加 ESO 扭矩。控制器通过 `ThreeNmpcControlAlgorithm` 接入；板端需提供已验证的输入 mapper，将协议状态与本地信号映射到各轴四维状态和参考。

串联腿的简化轴动力学现已从 NMPC 中独立出来，位于 `source/algorithms/eso_three_nmpc/serial_leg_dynamics.*`。该模块持有连续状态方程、扭矩输入增益、默认参数及状态 Jacobian；NMPC 层只通过接口调用模型，并独立维护目标函数权重、扭矩约束、预测时域与 QP 求解。当前默认参数仍是仿真占位系数，不是依据串联连杆几何、质量/惯量和驱动映射推导或辨识完成的整机动力学模型。旧 CAD `GroundModel + RK4` 复测 plant 也不等同于这个控制器预测模型，不能直接视为串联腿正式动力学。

QP 路径使用 acados dense-QP adapter、HPIPM 和 BLASFEO；当前构造的 36 变量凝聚 QP 仅支持单位矩阵盒约束形式。solver context 应在控制开始前初始化，控制周期只更新数据并求解。历史 `estimation.*`、`control_pipeline.*`、`state_reconstruction.*`、`scheduler.*` 等代码属于保留参考组件，并非当前选定的三轴 NMPC CMake 路径。

算法配置和验收基线见 [CONTROL_ALGORITHM_BASELINE.md](LINUXCODE/doc/control/CONTROL_ALGORITHM_BASELINE.md)。需注意该基线文档里的默认仿真参数与具体实验记录需按测试日期/配置一并阅读，不能把旧仿真参数当成当前板端已确认参数。

## 6. 验证现状与风险

- Linux 框架 CMake 单元测试覆盖 QP 解析解、边界、非法约束拒绝和三轴求解调用；它不验证真实传感器 ESO、目标板延迟或电机闭环。
- `LINUXCODE/tools` 的板端架构测试使用确定性算法替身，验证 AMP 解析、应用状态和数据路由，不验证真实 NMPC/电机行为。
- 2026-11-21 CAD `GroundModel + RK4` 复测中，0.1 s 零外扰与 0.2 s 扰动阶跃均完成求解，但姿态/yaw 闭环未通过；报告指出三轴模型状态定义、输入增益和关节力矩映射需要依据 CAD/辨识结果核验。
- 主机求解耗时只能作为主机诊断数据。交接目标板时序参考为完整周期 P99 <= 5 ms、P99.9 <= 8 ms、最大 <= 10 ms 且零超期，并需至少连续运行 1 小时记录分位数；当前并无满足这些条件的端到端板测结论。
- ESO `nominal_acceleration=0`、传感器快照尚未发送、Linux 本地单调时钟/AMP transport 未完成绑定、轴状态与参考 mapper 未经硬件确认，均是闭环接入前的实质性前置条件。

实验记录入口：[NMPC_ESO_HOST_VALIDATION_20261121.md](LINUXCODE/doc/experiments/three_nmpc/NMPC_ESO_HOST_VALIDATION_20261121.md)、[FRAMEWORK_GROUNDMODEL_RETEST_20261121.md](LINUXCODE/doc/experiments/three_nmpc/FRAMEWORK_GROUNDMODEL_RETEST_20261121.md)、[ACADOS_DENSE_QP_INTEGRATION_20261121.md](LINUXCODE/doc/experiments/three_nmpc/ACADOS_DENSE_QP_INTEGRATION_20261121.md)。

## 7. 推荐联调顺序

1. 明确并审计传感器坐标系、15 维状态定义、六路关节顺序/单位、参考语义，以及六路电机扭矩顺序/符号。
2. 在 RT-Thread 建立有明确采样时刻的状态快照；将 AtomIMU、关节位置/速度、机体状态与同一时刻 ESO 扰动组为 `0x12`。只有 ESO 初始化并有效更新后才置 bit3。
3. 接通 Linux AMP 接收适配器，记录接收端单调时间，验证 CRC、长度、序号回绕、超时、无效 bit3 和丢帧策略；先验证快照/ACK，不驱动执行器。
4. 实现并台架验证 `ThreeNmpcInputMapper` 和 motor writer 的索引、单位、符号、限幅、扭矩回执；保留 `DISABLED/ESTOP/FAULT` 输出禁止路径。
5. 在独立串联腿动力学模块中用连杆几何、质量/惯量、约束关系和驱动映射建立/辨识模型；用静态平衡和已知扰动阶跃验证后，再让 NMPC 使用该模型并扩展步态、转弯、腿长变化及长时测试。
6. 测量端到端时序、丢包、超期、ESO 更新有效率和安全停机行为；满足板端门槛后再声明部署验收通过。

## 8. 构建与测试入口

RT-Thread 固件在 [RTTHREADCODE/README.md](RTTHREADCODE/README.md) 描述的 SDK 环境中执行 `make` 或 `make dry-run`。Linux 框架与架构测试命令见 [LINUXCODE/README.md](LINUXCODE/README.md)；两者是不同构建目标，应分别报告构建/测试结果。固件生成不自动修改 Linux、U-Boot、设备树或镜像部署配置。

## 9. 关键文档索引

- [LINUXCODE 总说明](LINUXCODE/README.md)
- [Linux 系统架构](LINUXCODE/source/ARCHITECTURE.md)
- [AMP 协议](LINUXCODE/doc/amp/AMP_PROTOCOL.md)
- [BSP/AMP 交接](LINUXCODE/doc/amp/AMP_BSP_HANDOFF.md)
- [RT-Thread ESO 交接](LINUXCODE/doc/amp/RTTHREAD_ESO_HANDOFF.md)
- [控制算法基线](LINUXCODE/doc/control/CONTROL_ALGORITHM_BASELINE.md)
- [三轴 NMPC 说明](LINUXCODE/source/algorithms/eso_three_nmpc/README.md)
- [RT-Thread 固件说明](RTTHREADCODE/README.md)
