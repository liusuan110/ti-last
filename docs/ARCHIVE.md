# 工程演进与测试记录

F题工程从一套通用 CCS 模板开始，逐步加入电锁相、输出标定和光学控制。这里把这些阶段放在一起，方便理解代码为什么会留下不同的接口与实验脚本。

## 从基础驱动到电输入控制

最初的模板准备了时钟、串口、ADC/DMA、DAC、DDS、按键和编码器。随后应用逻辑围绕 F题展开，加入测频、PFD 反馈、直通继电器、图形模式和幅度档位。工程继续沿用 `ContestTemplate_2026` 名称，开发重点则转到 [User/App.c](../User/App.c)。

[MCU/DDS 联调记录](../F_CONTROLLER_BRINGUP.md) 保存了这一阶段的接口和上板顺序。2026-07-29，在 10 kHz 输入、0.5 V/div 和放大器接入的条件下，记录了四档幅度标定：

| Y 轴目标 | 同频 ASF | 二倍频 ASF |
| --- | ---: | ---: |
| 2 div / 1.0 Vpp | 116 | 112 |
| 4 div / 2.0 Vpp | 244 | 240 |
| 6 div / 3.0 Vpp | 364 | 364 |
| 8 div / 4.0 Vpp | 494 | 489 |

后续固件扩展为频率锚点和插值表，现行参数见 `gAmpAnchorHz/gAmpAnchorAsf`。这张表保留的是当时的单点测量条件与结果。

## CCD：先从一条扫描线取信息

第一条光学路线使用 TSL1401 竖直观察屏幕，配合 DAC 斜坡，将亮点位置、间距和数量转成频率及形状信息。它留下了 [TSL1401 驱动](../User/TSL1401.c)、[实时查看工具](../tools/ccd_live_view.ps1)和[要求5联调记录](../REQUIREMENT5_BRINGUP.md)。

2026-07-30 的一次 1 kHz 测试使用 40 ms 曝光和 5 ms 斜坡窗口，记录到最终识别 1000 Hz，总识别时间 2497 ms。这段记录也保存了像素中心、峰宽和规则点跨度，适合回看一维观测的调试过程。

[备赛库中的 F题报告](https://github.com/liusuan110/2026-Ti/tree/main/F%E9%A2%98%E8%AE%BE%E8%AE%A1%E6%8A%A5%E5%91%8A) 保留了较早的 CCD 方案草稿和 500 MHz DDS 时钟假设。

## UVC：从整幅轨迹测频与调形

后续改用 UVC 摄像头观察整个屏幕，通过轨迹提取、粗测频、100 Hz 候选搜索和 FTW 精调完成锁定。整幅图像也让直线、圆和“∞”可以采用各自的几何反馈量。

[lock_compensation_data.json](../DK2500_vision/lock_compensation_data.json) 保存了 2026-07-31 的 21 个锁点和频偏补偿数据，频段覆盖 1～100 kHz。该组旧流程的 `full_run_s` 为 18.398～27.617 s。

之后代码加入快速交接：把盲锁得到的 FTW 直接交给形状控制，由持续视觉反馈接着收敛和保持，减少阶段之间重复等待。当前流程从[按键服务](../DK2500_vision/requirement5_button_service.py)进入，运行方法见[复现指南](REPRODUCE.md)。

## 测试进度

现有记录包括四档幅度标定、CCD 单点识别和 21 个 UVC 频偏补偿点。加入快速交接后的版本，三种图形、全频段的完整控制时间与稳定性记录尚待补齐。要求 5 的目标是控制时间不超过 10 s、稳定至少 5 s；后续实验按按键到达标的完整流程计时，并同步更新报告中的测试表与结论。

本轮整理核对了源码、配置、脚本参数和文档链接，未重新进行固件构建、报告导出或硬件实验。

## 文档怎样接着维护

[方案与实现](SOLUTION.md) 解释设计，[复现指南](REPRODUCE.md) 记录操作步骤，本页追加带日期的实验与版本变化。写说明时先交代目的和工作方式，再给读者下一步；影响操作的条件放在对应步骤旁，实验状态集中记录在这里。

新实验附上源码提交、工具与硬件配置、频点、目标形状、原始输出及结论。这样，一段说明既能读懂，也能回到具体数据。

<details>
<summary>整理基线与历史入口</summary>

2026-11-01 开始内容整理，两个仓库保持可更新：

- [ti-last](https://github.com/liusuan110/ti-last)：F题完整工程与后续 UVC 方案。
- [2026-Ti](https://github.com/liusuan110/2026-Ti)：题库、训练工程、公共模块和备赛资料。

整理前 main 为 [0cfb725fba361fb958c790d13a437626d5d84539](https://github.com/liusuan110/ti-last/commit/0cfb725fba361fb958c790d13a437626d5d84539)，[原模板 README](https://github.com/liusuan110/ti-last/blob/0cfb725fba361fb958c790d13a437626d5d84539/README.md) 和[开题清单](../TOMORROW_CHECKLIST.md)记录了最初的环境与工作方法。

历史分支 `agent/requirement-5-optical-control` 的 HEAD 为 `8d0d37efae3fb47ba486f25b6cf486d46a0c6100`，是上述 main 的祖先，落后 16 个提交。

</details>
