# 非线性 ESO/NMPC 调参记录（2026-11-18）

本次检查针对 `GroundModel.f + RK4` 的非线性闭环 smoke，采样频率 500 Hz，NMPC 频率 100 Hz，控制器使用 direct-shooting，未调用旧 `linearize()` 或固定 A/B。

| 时长 | ESO 带宽 (roll/pitch/yaw) | horizon | 最大迭代 | 结果 |
|---:|---:|---:|---:|---|
| 0.05 s | 20/20/15 | 1 | 1 | 通过，25 samples |
| 0.10 s | 10/10/8 | 1 | 1 | 通过，50 samples |
| 0.10 s | 10/10/8 | 2 | 1 | 通过，50 samples |
| 0.10 s | 15/15/10 | 1 | 1 | 通过，50 samples |
| 0.10 s | 20/20/15 | 1 | 1 | 通过，50 samples |
| 0.20 s | 10/10/8 | 1 或 2 | 1 | `Closed-chain predictor outside CAD closure branch` |
| 0.20 s | 8/8/6 | 1 | 1 | `Closed-chain predictor outside CAD closure branch` |

当前调试起点采用 `ESO=10/10/8`、`horizon=2`、`max_iterations=1`。这组在 0.1 秒内的 pitch/yaw 最大绝对角度约为 `0.0757/0.0183 rad`，ESO 扰动峰值约为 `0.0038/0.2421/0.1400 rad/s²`（roll/pitch/yaw）。

0.2 秒失败发生在 CAD 闭链求解器的 branch 检查，而不是 QP 组装测试；后续需要在 Isaac 完整浮动基座模型中重新辨识输入增益、扭矩限幅/变化率和接触切换，或给预测器增加 branch-safe 状态约束后再延长时长。现阶段所有 smoke 报告继续保持 `deployment_accepted=false`。

复现命令：

```bash
python3 tools/run_nonlinear_decoupled_smoke.py \
  --seconds 0.10 --eso-bandwidth 10 10 8 \
  --horizon 2 --max-iterations 1 \
  --output /tmp/nonlinear_tuned_trace.csv
```
