# 2026 F题｜李萨如图形显示控制装置

本仓库归档 2026 年 F 题的完整工程、方案演进和设计报告：以 **MSPM0G3507 + AD9959** 为控制核心，实现直通、同频正交、二倍频和幅度档位控制，并保留要求 5 的 UVC 摄像头视觉自动控制代码。

年度备赛资料、历年题目、训练工程和通用模块请到 [2026-Ti 备赛资产库](https://github.com/liusuan110/2026-Ti)。本仓库保持可更新；“归档”指内容整理和版本留痕。

## 从哪里开始

| 目的 | 入口 |
| --- | --- |
| 看系统思路、方案取舍与代码对应关系 | [方案与实现](docs/SOLUTION.md) |
| 导入 CCS、核对接线、启动视觉流程 | [复现指南](docs/REPRODUCE.md) |
| 看归档基线、历史资料与验证边界 | [归档说明](docs/ARCHIVE.md) |
| 阅读报告、原理图与插图 | [F题设计报告](F题设计报告/readme.md) · [报告 PDF](F题设计报告/电赛F题_设计报告.pdf) |
| 修改主控程序或外设配置 | [User/App.c](User/App.c) · [SysConfig](ContestTemplate_2026.syscfg) |
| 阅读最终视觉主流程 | [按键服务](DK2500_vision/requirement5_button_service.py) → [流程编排](DK2500_vision/requirement5_target.py) |

## 工程组成

- **MCU 固件**：根目录为 CCS 工程，工程名沿用 `ContestTemplate_2026`。应用逻辑在 `User/App.c`，驱动包括 AD9959、ADC/DMA、DAC、按键、编码器、RGB 和声光提示。
- **视觉控制**：`DK2500_vision/` 包含摄像头服务、斜坡轨迹测频、FTW 精调、三种图形视觉反馈和一键流程。
- **报告与硬件**：`F题设计报告/` 包含 Typst 源码、已有 PDF、原理图和框图。
- **早期探索**：TSL1401 线阵驱动、`tools/ccd_live_view.ps1` 及旧联调文档保留，用于理解 CCD 路线到 UVC 路线的演进。

## 两条工作链路

**要求 1～4**：电输入测频 / PFD 反馈 → MCU 控制 → AD9959 → 输出放大与继电器 → 示波器 Y 轴。

**要求 5**：信号源仅接示波器 X 轴 → 摄像头拍屏 → 斜坡测频与盲锁 → 精确 FTW 交接 → 目标图形视觉闭环 → MCU / AD9959 输出 Y 轴。最终 UVC 流程由物理按键和 Python 按键服务启动；固件中的 `fauto` 仍属于本地 CCD 路线。

## 环境与复现状态

工程配置记录：CCS 12.7、TI ARM LLVM 4.0.2 LTS、MSPM0 SDK 2.10.00.04、SysConfig 1.26.2；MCU 为 MSPM0G3507，PA6 需要外部 40 MHz 时钟。DDS 的 25 MHz 系统时钟与 MCU 外部时钟是不同配置。

Python 视觉代码使用 OpenCV、NumPy 和 pyserial，仓库未锁定依赖版本。相机画幅、ROI、串口及幅度参数需要按实物确认，详见复现指南。

仓库保存了幅度标定和部分光学测试记录；本次整理仅核对源码、配置和文档，未重新编译或上板测试。要求 5 的最终版本仍需补齐全频段、三模式的控制时间与稳定性日志，不能把已有代码或报告结论等同于全指标验收。

## 历史记录

[F_CONTROLLER_BRINGUP.md](F_CONTROLLER_BRINGUP.md)、[REQUIREMENT5_BRINGUP.md](REQUIREMENT5_BRINGUP.md) 和 [TOMORROW_CHECKLIST.md](TOMORROW_CHECKLIST.md) 保留原始阶段内容。遇到旧的“待实现”、按键定义和模板资源占用数据，应结合当前源码与[归档说明](docs/ARCHIVE.md)判断。
