# RK3568 RT-Thread 控制固件

这是从 SDK 中现有 `rtos/bsp/rockchip/rk3568` 代码整理出的独立工作目录。代码仍按实际存在的功能分层，保持 RT-Thread CPU3 AMP、电机/CAN、AtomIMU、SPI3、UART4、DMA 和共享内存协议的原有逻辑。

## 目录

```text
application/              RT-Thread 入口、线程、DM 运动页、电机清单注册
modules/                  已存在的控制器、daemon、AtomIMU、DJI/DM/AMP 电机服务
bsp/
  board/                  RK3568 启动、GIC/heap/板级寄存器
  amp/                    IPC、共享内存、诊断和 AMP 协议头
  can/                    CAN 接收分发
  dma/                    PL330 DMA
  diag/                   运行诊断
  dwt/                    时间基准
  periph/                 CAN/SPI/PWM/BMI088 公共外设接口
  spi/                    SPI3/BMI088
  usart/                  UART4/RS485
  log/                    RT 日志接口
config/                   已有电机清单
link.lds                  RK3568 CPU3 链接脚本
rtconfig.py/rtconfig.h    RT-Thread 配置和工具链
```

目录中没有实际代码的 `gimbal`、`chassis`、`shoot`、I2C、ADC、USB、STM32 `Core/Drivers/Middlewares`，因此没有伪造这些模块；后续有真实实现时再按同一边界加入。工程只使用 RT-Thread，不包含 FreeRTOS。

## 地址和依赖

链接地址保持板端已经使用的真实窗口：CPU3 固件 `0x03000000`，大小 `0x00200000`；AMP 共享内存仍是 `0x07800000`，各协议页地址由 `bsp/amp` 中的现有协议头定义。不要单独改 `link.lds`，还需要同步 Linux DTS、AMP 驱动和 FIT 配置。

RT-Thread 内核、libcpu 和组件继续来自 SDK 的 `rtos/`，默认由 `rtconfig.py` 自动解析到：

```text
/home/yeht/SDK/rk356x-linux-2026051515/rtos
```

也可以显式设置 `RTT_ROOT`、`RTT_EXEC_PATH` 和 `RTT_CC_PREFIX`，不会依赖当前 shell 的工作目录。

## Linux AMP 应用协议

`application/amp_application.c` 已接入 Linux 侧的 AMP v0.1 线帧：同步字节为
`A5 5A`，整数小端，float16 在解析后立即转换为 `float`，CRC 为
CRC-16/CCITT-FALSE。`COMMAND_SETPOINT (0x01)` 通过校验、序号、有限值和状态字
检查后才更新应用目标，并返回固定 12 字节的 `COMMAND_ACK (0x02)`；重复、乱序、
坏 CRC、非法状态和超长帧不会改写目标。连续 50 ms 没有有效命令会自动回到
`DISABLED/STOP`，并通过 `MotorSetOutputInhibit()` 关闭电机输出。现有文本诊断和
64 字节 RTT 回显仍保留在同一 IPC 接收线程中。

当前代码库没有云台、底盘或发射控制实现，所以 AMP 命令只进入应用命令状态和
安全门控，不虚构这些控制模块；后续模块可通过 `amp_application_get_command()`
读取已验证的 SI 单位目标。

## Global ESO 移植状态

`modules/algorithm/global_eso.c` 按 `LINUXCODE/rtthread_reference/decoupled_control.*`
移植 Global ESO 数值实现，`periph` 线程在 AtomIMU 的欧拉角和陀螺仪数据均有新帧时更新。
默认带宽为 roll/pitch/yaw=`35/35/25`，扰动限幅为 `30/30/20 rad/s^2`，非线性参数为
`alpha=0.5`、`delta=0.01`。角度使用 ATOM 的 pitch/roll/yaw 顺序映射到 ESO 的
roll/pitch/yaw；`nominal_acceleration` 暂设为零，因为当前 RT 工程没有对应的刚体模型项。

该移植目前只运行并保存 ESO 估计，不代表 AMP `0x12` 快照已发送。当前 RT 工程尚无
完整 CAD 15 维状态、六路关节位置/速度和同步采样快照构造器；在这些真实量接入前，不能
用默认零值拼帧或置 `flags.bit3`。Linux 侧 `0x12` 解码/架构测试验证的是接收边界，板端
发送链路仍待完整传感器状态来源接入。

板测建议门槛见 `LINUXCODE/doc/amp/RTTHREAD_ESO_HANDOFF.md`：ESO 2 ms 周期、执行时间
P99 ≤ 1 ms 且零漏周期；完整控制周期 P99 ≤ 5 ms、P99.9 ≤ 8 ms、最大值 ≤ 10 ms 且零漏周期；
连续运行至少 1 小时并记录 P50/P99/P99.9/最大值/超期数。当前 ESO 位于每 5 ms 运行的
`periph` 线程，因此尚不满足 2 ms 调度要求；真实 NMPC 适配器未接入，完整周期也尚不能板测。

M3508 的反馈已按输出轴换算：减速比为 `20.3175`（替换旧的 `3591/187`），
位置和速度都除以该值，型号最大扭矩保留为 `3 Nm`。GM6020/M2006 仍使用
`1.0` 换算比。

M3508/C620 的电流指令范围是 `±16384` 对应 `±20 A`。驱动提供两种力矩换算：
输出轴力矩为 `0.3 × (20/16384) × current_cmd`，转子侧力矩为
`(0.3/20.3175) × (20/16384) × current_cmd`；后者用于和转子转速计算功率。
如果功率模型的速度仍使用 RPM，交叉项系数为
`(20/16384) × (0.3/20.3175) / 9.55 = 1.88737271e-6`。
AMP 的 `TORQUE` 命令使用输出轴 Nm，RT-Thread 会反算为 C620 电流计数；
M3508 默认按 `3 Nm` 限制，对应约 `8192` 个电流计数。`CURRENT` 命令仍表示原始电流计数。

## 编译

在本目录执行：

```sh
make                 # 生成 rtthread.elf 和 rtthread.bin
make dry-run         # 只检查 SCons 收集的源码和链接命令
make clean
```

也可以直接调用：

```sh
RTT_ROOT=/home/yeht/SDK/rk356x-linux-2026051515/rtos \
RTT_EXEC_PATH=/home/yeht/SDK/rk356x-linux-2026051515/prebuilts/gcc/linux-x86/arm/gcc-arm-none-eabi/bin \
scons -C /home/yeht/SDK/CODE/RTTHREADCODE -j4
```

生成的裸固件还需要按 SDK 的 AMP/FIT 流程部署；这个目录的 Makefile 不会偷偷修改 Linux、U-Boot 或镜像分区。
