# RK3568 串联腿轮式机器人控制代码

仓库包含两部分：

- `LINUXCODE/`：Linux 控制框架、AMP 协议、串联腿轴动力学模块、三轴 NMPC 与主机测试。
- `RTTHREADCODE/`：RK3568 CPU3 RT-Thread 固件、板级外设、电机服务、AMP IPC 与 Global ESO。

两部分通过 AMP 协议协同工作，构建方式和依赖不同，请分别阅读各自目录下的 README。系统职责、接口、当前集成状态和动力学模型边界见 [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)。
