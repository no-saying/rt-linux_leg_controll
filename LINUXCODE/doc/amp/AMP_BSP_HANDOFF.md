# AMP/BSP 板端交接说明

## 当前工程负责的部分

- AMP 线帧常量和消息类型；
- float16 字段布局和校验规则；
- 命令、ACK、遥测的固定字段定义；
- 应用层状态机对 `vehicle_state` / `motion_state` 的解释；
- 固定格式帧 codec、CRC 校验、命令边界检查、序号和命令超时处理。

## 板端工程负责的部分

板端后续在 RTThread 工程中实现：

- AMP 内核通信通道的收发回调；
- DMA、中断、缓存和内核线程；
- 遥控器数据读取和发送周期；
- Global ESO 的周期任务、输入时间对齐、初始化/复位和输出有效性；
- 把 roll/pitch/yaw 三轴 ESO 集总扰动写入 `0x12` 的 `eso_disturbance[3]`，并在观测有效时置 flags.bit3；
- 接收 ACK、故障回报和链路重连；
- 硬件时间戳、看门狗和故障复位；
- 电机、IMU、编码器和电源管理驱动。

当前仓库不实现 RT-Thread 内核。保留的 `arm_mpc_rl_p112_nmpc_20261118/cpp_runtime/decoupled_control.*`
`source/algorithms/eso_three_nmpc/decoupled_control.*` 含 `GlobalEso` 实现及三个
NMPC block，供 RT-Thread 交接和应用层接入参考；板端应在 RT-Thread 采集传感器
后更新 ESO，再把传感器和 ESO 结果打进同一 `0x12` 帧。不要在本控制框架中再创建
第二个 ESO。板端把完整 AMP 命令/传感器帧和本地单调接收时间交给
`ControlRuntime::ingestAmpBytes`，并在每次控制周期把当前本地单调时刻传给
`tick(now_us)`，再把有效的六路输出交给电机驱动。不能用最后一帧的接收时间代替
`now_us`，否则链路静止时无法检测超时。
USB 用于其他外设，既不进入这条控制链，也不作为 AMP 承载。

## 固定内存约束

板端协议路径必须使用固定帧缓冲、固定长度消息槽和静态 topic。禁止在控制周期
中使用 malloc/free、std::vector、std::string、std::function、线程创建或阻塞式
文件/串口 IO。日志通过静态环形缓冲转交低优先级线程。

## 接入顺序

1. 在 RT-Thread 中移植/运行保留的 `GlobalEso`，确认其测量、输入模型项、采样周期和复位条件；
2. 将三轴 ESO 集总扰动及有效位与对应传感器样本写入 AMP `0x12`，按 `AMP_PROTOCOL.md` 发送命令帧和传感器帧；
3. 把完整 AMP 帧和接收时刻传入 `ControlRuntime`，确认 NMPC 输入收到同帧 ESO 观测值；
4. 只有 `ControlOutput.valid` 为真时才写电机，急停、故障和命令超时禁止输出；
5. 运行 `tools/control_stack_board_test` 的架构自测，再接入真实 NMPC 和其他本地感知算法；
6. 记录固件版本、协议版本、ESO/NMPC 周期耗时、超时和故障行为。

板端完成后应提供协议版本、承载方式、最大延迟、丢包行为、时间戳来源和故障
复位流程，作为本文件的附录，不修改算法层接口。
