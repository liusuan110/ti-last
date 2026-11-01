# 2026 F题｜李萨如图形显示控制装置

这个装置用来控制示波器 X-Y 模式下的李萨如图形。MSPM0G3507 驱动 AD9959 调整输出频率、相位和幅度，使屏幕显示对角线、圆和“∞”，并提供 2/4/6/8 div 四档幅度。常规模式利用电输入和鉴相反馈；要求 5 则通过 UVC 摄像头观察屏幕，完成测频和图形调整。

仓库包含主控固件、视觉程序、原理图和设计报告，也保留了从线阵 CCD 到整屏视觉的探索过程。年度题库、训练工程和通用模块资料集中在 [2026-Ti 备赛资产库](https://github.com/liusuan110/2026-Ti)。

## 先看什么

想理解设计，先读[方案与实现](docs/SOLUTION.md)：从图形需要什么控制量，讲到电锁相、光学测频和形状保持。

准备搭建时，先看[硬件分工与连接](docs/HARDWARE.md)，再进入[复现指南](docs/REPRODUCE.md)：按顺序导入 CCS、检查串口、启动相机和按键服务，每一步都列出可以观察的现象。

需要原理图和正式叙述，打开[设计报告](F题设计报告/readme.md)；想回看开发过程和实验数据，阅读[工程演进与测试记录](docs/ARCHIVE.md)。

## 装置怎样工作

示波器 X 轴接信号源，Y 轴接装置输出。频率比决定图形有几个环，相位决定它怎样展开，幅度决定屏幕上的跨度。AD9959 把这三个量变成可调参数，主控根据电测或图像反馈持续修正输出。

要求 1～4 从电输入获得频率和相位信息。要求 5 只通过屏幕观察未知信号：先用已知窗口的斜坡辅助测频，再切换到 DDS 输出，保持目标形状。两种方式共用同一套主控和 DDS。

## 硬件与控制架构

主控板之外，系统还包括 AD9959、输入调理与 PFD、输出放大与继电器，以及信号源和示波器。光学控制再加入 UVC 摄像头与 DK/Linux 主机。[硬件说明](docs/HARDWARE.md)按模块列出分工和连接。

### 要求 1～4：电输入与 DDS

[![电输入与 DDS 控制链路](docs/images/electrical-control.svg)](docs/images/electrical-control.svg)

电输入提供测频与鉴相信息，DDS 产生 Y 轴信号。图中保留主信号链，CH1 参考、PFD 反馈和引脚对应关系在[硬件说明](docs/HARDWARE.md#电输入模式的连接)中展开。

### 要求 5：从屏幕取得反馈

[![UVC 光学控制链路](docs/images/optical-control.svg)](docs/images/optical-control.svg)

摄像头与 Python 程序把屏幕图像变成频率和形状反馈；Y 轴先接收测频斜坡，再接收 DDS 图形输出。两张图都可点击查看原尺寸，完整引脚表见[复现指南](docs/REPRODUCE.md#接线速查)。

## 软件从哪里读

| 部分 | 入口 | 阅读重点 |
| --- | --- | --- |
| 主控应用 | [User/App.c](User/App.c) | 主循环、按键、命令解析、电锁相和输出模式 |
| 外设与时序 | [User](User/) · [SysConfig](ContestTemplate_2026.syscfg) | AD9959、ADC/DMA、DAC、1 ms 时基和引脚配置 |
| 一键视觉流程 | [按键服务](DK2500_vision/requirement5_button_service.py) → [目标编排](DK2500_vision/requirement5_target.py) | 从物理按键到测频、锁定和形状控制 |
| 视觉算法 | [盲锁](DK2500_vision/fast_phase_lock.py) · [形状控制](DK2500_vision/shape_control.py) | 频率候选、精确控制字交接和持续反馈 |
| 报告与硬件资料 | [F题设计报告](F题设计报告/readme.md) | 系统方案、原理图、插图和已有 PDF |

主控工程名沿用 `ContestTemplate_2026`。从 [ContestTemplate_2026.c](ContestTemplate_2026.c) 进入初始化与主循环；视觉程序从按键服务向下阅读，更容易看清各脚本之间的数据流。
