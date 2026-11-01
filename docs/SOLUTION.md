# F题方案与实现

## 目标如何转成控制量

题目要求在示波器 X-Y 模式下形成对角线、圆和“∞”，并设置 Y 轴 2/4/6/8 div 幅度。核心控制量是两路信号的频率比、相位差和幅度比：同频同相形成对角线；同频正交、等幅形成圆；二倍频配合相位调整形成对称“∞”。0.5 V/div 条件下，2/4/6/8 div 对应 1/2/3/4 Vpp。

要求 1～4 可以利用电输入与鉴相反馈；要求 5 切断装置与信号源的电气连接，因此必须从示波器图像获得未知频率和形状误差。1～100 kHz、100 Hz 步进、控制不超过 10 s、稳定至少 5 s 是题目指标，实际达成情况见[验证边界](ARCHIVE.md)。

## 电测与 DDS 链路

[User/App.c](../User/App.c) 组织命令、按键、测频、锁相与输出模式。PA16 用于电输入测频，PA27 接 PFD 反馈；[AdcCapture](../User/AdcCapture.c) 和中断模块提供采样与时序支持。AD9959 驱动负责频率字 FTW、相位字 POW 和幅度字 ASF，CH0 输出最终 Y 轴信号，CH1 保留为外部 PFD 参考用途。

DDS 当前系统时钟为 25 MHz，频率字按 `FTW = round(f_out × 2^32 / f_sys)` 计算。相位为 14 位、幅度为 10 位；数字分辨率不能替代实物频响、输出增益和漂移标定。

PA13 控制直通 / DDS 继电器：低电平直通，高电平选择 DDS 放大输出。固件保留输出切换和错误状态处理。幅度档位以当前 `gAmpAnchorHz`、`gAmpAnchorAsf` 及插值代码为准；报告中的 10 kHz 表是一个历史标定截面。更换放大器、参考时钟或电源后需要重新标定。

## 要求 5：从 CCD 到 UVC

早期方案使用 TSL1401 竖直扫描屏幕，配合 PA15 锯齿波把空间交点转换为时间信息。它需要处理峰值重叠、像素量化、曝光和窗口边界等问题。仓库保留 [TSL1401 驱动](../User/TSL1401.c)、[线阵查看工具](../tools/ccd_live_view.ps1) 和[阶段实测记录](../REQUIREMENT5_BRINGUP.md)。

后续方案改用 UVC 摄像头采集整幅屏幕，通过 Python/OpenCV 获得更多几何信息。最终主流程如下：

1. [mjpeg_server.py](../DK2500_vision/mjpeg_server.py) 从 UVC 相机取得图像，提供 `/frame.jpg` 和 MJPEG 流。
2. [ramp_frequency.py](../DK2500_vision/ramp_frequency.py) 等模块利用窗口化斜坡轨迹估计未知输入频率，结合粗估、100 Hz 候选搜索和精炼缩小范围。
3. [fast_phase_lock.py](../DK2500_vision/fast_phase_lock.py) 完成盲锁与 FTW 微调，将准确的频率控制字交给后续阶段。
4. [requirement5_target.py](../DK2500_vision/requirement5_target.py) 传递锁定 FTW 与初始相位，启动 [shape_control.py](../DK2500_vision/shape_control.py)；避免用舍入后的频率重新生成 DDS 参数。
5. 目标控制按图形几何误差调相并补偿漂移：直线关注线宽和间隙；圆关注轴比、拟合与角度覆盖；“∞”关注中心穿越及叶瓣对称性。
6. [requirement5_button_service.py](../DK2500_vision/requirement5_button_service.py) 在空闲时监听 MCU 按键事件，将串口交给目标流程；MCU 复位后返回等待下一次按键。

`--fast-handoff` 用于减少阶段间重复保持验证。视觉伺服接手后仍需继续收敛和保持，因此“参数已就绪”的时间不等于题目要求的完整控制时间。

## 按键与命令为何分两条路线

PA16 有电输入时：UP 启动自动测频 / 电锁相，等待锁定后 MID 圆、RIGHT “∞”；LEFT 直通，DOWN 切幅度档位，编码器调相位、按下回直通。

PA16 无输入时：LEFT / MID / RIGHT 发送 `F_EVENT requirement5_start=line|circle|infinity`，由 Python 按键服务启动完整 UVC 流程。

`fauto line|circle|infinity` 仍调用 MCU 的本地 CCD 控制；它不是最终 UVC 盲锁管线的命令入口。调试 UVC 单目标请使用 Python 的 `requirement5_target.py`，详见[复现指南](REPRODUCE.md)。

## 阅读顺序

先看本说明与 [README](../README.md)，再从 `App_init/App_loop`、`requestDKAutoMode` 和串口解析阅读固件；视觉部分从按键服务、目标编排、盲锁、形状控制逐层进入。其他候选搜索、视频漂移和诊断脚本作为实验工具保留，不能仅凭文件名判断它是最终默认流程。

完整报告与电路图见 [F题设计报告](../F题设计报告/readme.md)。备赛库中的 CCD 报告是更早阶段的草稿，反映方案探索，不与这里的 UVC 报告相互覆盖。
