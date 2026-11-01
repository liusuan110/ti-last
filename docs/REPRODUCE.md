# 工程复现指南

本文依据归档时的工程配置、脚本参数和源码整理。所列启动命令是复现入口，本次文档整理未重新进行编译、相机运行或上板测试。

## 1. MCU 工程

安装工程记录的工具版本：CCS 12.7、TI ARM LLVM 4.0.2 LTS、MSPM0 SDK 2.10.00.04、SysConfig 1.26.2。根目录已有 `.project/.cproject/.ccsproject`，从 CCS 的现有工程导入入口选择仓库根目录，导入工程名为 `ContestTemplate_2026`，随后全量重建。

目标器件为 MSPM0G3507 / LQFP-48，当前调试器配置为 SEGGER J-Link。PA6 需要外部 40 MHz HFCLK，CPU 配置为 80 MHz；换板后应先核对时钟。修改外设先编辑 [ContestTemplate_2026.syscfg](../ContestTemplate_2026.syscfg)，重新生成配置，不直接修改生成目录。

旧模板 README 的 Flash / SRAM 数值不代表当前 F 题固件。请保存本次实际构建的资源占用、编译器版本和固件文件，用于后续复测。

## 2. 当前接线

| 用途 | 引脚 / 说明 |
| --- | --- |
| 电输入测频 / PFD 反馈 | PA16 / PA27；进入 MCU 前按电路完成调理 |
| DAC 窗口斜坡 | PA15 |
| DDS SCLK / SDIO0 / CS / UPDATE / RESET | PA8 / PA9 / PB3 / PB2 / PA7 |
| DDS 最终输出 / PFD 参考 | CH0 / CH1 |
| 直通继电器 | PA13，低直通、高 DDS |
| 外部有源蜂鸣器 + LED | PA1，高有效，经 S8050 驱动 |
| UART0 调试与视觉控制 | TX PA10 / RX PA11，115200-8-N-1 |
| UART2 屏幕预留 | TX PB17 / RX PB18 |
| RGB R / G / B | PA2 / PA3 / PA4 |
| LEFT / DOWN / RIGHT / UP / MID | PB6 / PB7 / PB8 / PB9 / PB14 |
| 编码器 A / B / 按下 | PB15 / PB16 / PA12 |
| SWDIO / SWCLK / 外部 40 MHz | PA19 / PA20 / PA6 |
| 早期 TSL1401 | AO PA14 / CLK PA17 / SI PA18，保留实验路线 |

UART0 与 USB-TTL 交叉连接 TX/RX 并接地。AD9959 当前 25 MHz 系统时钟配置见 [AD9959.h](../User/AD9959.h)，不要套用备赛库 CCD 草稿中的 500 MHz 参数。

要求 5 测试时，未知信号源仅接示波器 X 轴，装置电输入 PA16 断开；DDS / 斜坡经实物切换链路接 Y 轴，相机观测整个屏幕。不能通过调试参数把未知输入频率预先传给自动流程。

## 3. 基础串口检查

先关闭占用同一串口的其他程序，以回车或换行结尾发送：

```text
fhelp
fstatus
fmode thru
```

已知输入条件下可以用 `ffreq 10000`、`fmode same|quad|double`、`fdiv 2|4|6|8` 等调试 DDS。每次只发送一个具体命令，`|` 表示可选值。`ffreq` 是人工设频，不能用它证明自动测频或要求 5 达标。当前电锁相关命令以 `fhelp` 输出和 `User/App.c` 的 `plock` 实现为准。

## 4. Python 与摄像头

以下示例面向 DK/Linux 的 Python 3.10+ 环境，先进入视觉目录：

```bash
cd DK2500_vision
python3 -m pip install opencv-python numpy pyserial
```

依赖版本未锁定；装好后记录实际版本。串口默认 `/dev/ttyUSB0`，相机默认 `/dev/video0`；使用实际设备名，并保证串口用户具有访问权限。

**先校准画幅与 ROI。** [vision_config_mf500.json](../DK2500_vision/vision_config_mf500.json) 的 ROI 为 `x=150,y=160,w=830,h=610`，至少需要覆盖 980×770 的图像。相机服务默认 640×480，不能直接配合这份 ROI 使用。按相机实际支持的画幅调整采集参数和配置中的 `roi/frequency_roi`；移动相机或改变缩放后重新设置 ROI、HSV 阈值及频率比例系数。

例如，仅在相机支持 1280×960 时可以尝试：

```bash
python3 mjpeg_server.py --device /dev/video0 --width 1280 --height 960 --fps 60 --fourcc MJPG
```

60 fps 是请求值，应以启动时 `camera_active` 输出的实际画幅和帧率为准。浏览器打开 [相机画面](http://127.0.0.1:8080/) 或 [单帧图像](http://127.0.0.1:8080/frame.jpg)，确认屏幕轨迹清晰、ROI 在画面内且无明显反光。

## 5. 要求 5 一键流程

保持相机服务运行，在另一个终端进入同一视觉目录：

```bash
python3 requirement5_button_service.py --serial /dev/ttyUSB0 --config vision_config_mf500.json --url http://127.0.0.1:8080/frame.jpg --output-dir ./target_output
```

PA16 无输入时，按 LEFT / MID / RIGHT，分别启动直线 / 圆 / “∞”。服务的默认幅度字为 489，`stable-seconds` 为 5，不能把这些默认参数当作所有硬件上的校准结果。

按键服务向目标流程传递 `servo-duration=0`，表示视觉反馈持续运行；它不会在图形第一次成功后自动退出。MCU 复位会让活动流程返回空闲监听。空闲服务与活动子进程轮流持有串口，避免另开串口监视器争用设备。

独立调试某一目标、希望流程在有限时间后返回时：

```bash
python3 requirement5_target.py --serial /dev/ttyUSB0 --config vision_config_mf500.json --target line --servo-duration 15 --output-dir ./target_output
```

将 `line` 替换为 `circle` 或 `infinity` 可测试其他目标。`fauto` 是固件本地 CCD 路线，不代替上述 UVC 流程。

## 6. 留下可追溯的复测结果

每次记录源码提交、固件、工具 / Python 版本、相机画幅和实际帧率、ROI、DDS 时钟、放大器与幅度参数、输入频点和目标形状。

保存串口日志、Python 输出、目标图像，以及从按键到达标的完整耗时；继续观察至少 5 s，记录漂移、抖动和失败重试。覆盖频段边界、典型频点和三种目标，单独检查幅度档位及断线 / 复位行为。现有标定数据和阶段耗时不能替代这一轮完整验收。
