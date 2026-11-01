# 把工程跑起来

复现分成三段：先确认主控和 DDS 能工作，再跑电输入模式，最后接入相机完成光学自动控制。按这个顺序推进，每段都能留下一个清楚的观察结果。

## 先取到工程，并确定在哪台主机运行

```bash
git clone https://github.com/liusuan110/ti-last.git
cd ti-last
```

也可以在 GitHub 选择 Code → Download ZIP 后解压。下文的“仓库根目录”指同时包含 `ContestTemplate_2026.syscfg`、`User/` 和 `DK2500_vision/` 的这一层。

开发电脑运行 CCS，连接 J-Link；DK/Linux 主机连接 UVC 摄像头和 USB-TTL，运行后面的 Python 命令。两边使用同一版本源码，硬件分工见[连接说明](HARDWARE.md)。

## 1. 导入主控工程

工程记录的环境如下：

| 项目 | 配置 |
| --- | --- |
| 主控 | MSPM0G3507，LQFP-48 |
| IDE / 编译器 | CCS 12.7 / TI ARM LLVM 4.0.2 LTS |
| SDK / 配置工具 | MSPM0 SDK 2.10.00.04 / SysConfig 1.26.2 |
| 调试器 | SEGGER J-Link |
| MCU 时钟 | PA6 输入外部 40 MHz，CPU 80 MHz |
| DDS 系统时钟 | 25 MHz，见 [AD9959.h](../User/AD9959.h) |

在 CCS 中导入仓库根目录的现有工程，工程名为 `ContestTemplate_2026`，执行全量重建。需要调整引脚或外设时，编辑 [ContestTemplate_2026.syscfg](../ContestTemplate_2026.syscfg)，由 SysConfig 重新生成配置。

先连接调试器和 UART0，波特率设为 115200-8-N-1。烧录并复位后，串口应出现：

```text
2026 F Lissajous MCU READY
MSPM0G3507 / 80 MHz / SDK 2.10.00.04
```

随后发送 `fhelp` 查看命令，发送 `fstatus` 查看当前模式。看到启动信息和命令响应后，再继续连接信号链路。

## 2. 检查电输入和 DDS 输出

先按[接线表](#接线速查)接好输入调理、鉴相、DDS 和输出放大链路。示波器切到 X-Y 模式，X、Y 通道灵敏度设为 0.5 V/div，信号源先选一个已知频点，例如 10 kHz。

串口命令以回车或换行结束。先发送 `fmode thru`，在直通状态下把输入调成 8×8 div 的对角线。在 0.5 V/div 下，8 div 对应 4 Vpp，以示波器实测跨度为准，随后固定信号源幅度。

再切到同频 DDS 输出：

```text
ffreq 10000
fdiv 8
fmode same
```

确认 DDS 输出频率与设定一致、Y 轴跨度随档位变化。完成这一段后，按 UP 让主控自动测频并建立电锁相；锁定后按 MID 看圆，按 RIGHT 看“∞”。DOWN 依次选择 2/4/6/8 div，编码器旋转用于调相位，LEFT 或编码器按下回到直通。

幅度表和相位起始值保存在 [User/App.c](../User/App.c) 中的 `gAmpAnchorHz/gAmpAnchorAsf`、`gCirclePhaseAnchorHz/gCirclePhaseAnchorWord`。换用放大器或参考时钟后，在实际输出链路上重新测量并更新这些参数。

## 3. 让相机先看清屏幕

在连接 UVC 摄像头和 USB-TTL 的 DK/Linux 主机上，使用 Python 3.10+。从这台主机上的仓库根目录进入视觉目录并安装依赖：

```bash
cd DK2500_vision
python3 -m pip install opencv-python numpy pyserial
```

[vision_config_mf500.json](../DK2500_vision/vision_config_mf500.json) 记录了轨迹颜色、取景区域 ROI 和测频比例。现有 ROI 为 `x=150,y=160,w=830,h=610`，图像需要至少覆盖 980×770。先选择相机支持的画幅，再同步调整 `roi` 和 `frequency_roi`；相机服务默认的 640×480 画幅需要配一份更小的 ROI。

若相机支持 1280×960，可以从下面的配置开始：

```bash
python3 mjpeg_server.py --device /dev/video0 --width 1280 --height 960 --fps 60 --fourcc MJPG
```

终端会输出 `camera_active`，其中列出相机实际采用的画幅、格式和帧率。在这台主机上打开[实时画面](http://127.0.0.1:8080/)或[单帧图像](http://127.0.0.1:8080/frame.jpg)，把屏幕轨迹完整放进 ROI，调整对焦、位置和曝光，使轨迹清晰、背景稳定。从另一台电脑查看时，把地址中的 `127.0.0.1` 换成 DK 主机 IP；主机内脚本的 `--url` 仍使用原地址。

移动相机或改变缩放后，重新确定 ROI，并检查 HSV 阈值与 `frequency_scale`。先把这一帧看对，后面的测频和形状评分才有稳定输入。

## 4. 从按键启动光学自动控制

从电输入模式切换过来时，先发送 `fmode off` 或复位 MCU，结束电锁相状态。随后将未知信号源只接到示波器 X 轴，断开装置 PA16 的电输入。按[光学模式接入表](HARDWARE.md#光学模式y-轴在两个阶段接收不同信号)检查 Y 轴的斜坡和 DDS 两条信号路径。

保持相机服务在第一个终端运行，关闭占用 UART0 的串口终端。在同一台 DK/Linux 主机的第二个终端进入 `DK2500_vision/`，启动按键服务：

```bash
python3 requirement5_button_service.py --serial /dev/ttyUSB0 --config vision_config_mf500.json --url http://127.0.0.1:8080/frame.jpg --output-dir ./target_output
```

看到 `BUTTON_SERVICE_IDLE` 后，按 LEFT / MID / RIGHT，分别选择直线、圆和“∞”。终端会出现对应的 `BUTTON_SERVICE_START target=...`，随后运行测频、频率锁定和形状控制。整个过程从图像识别未知频率。

默认幅度字为 489，稳定观察参数为 5 s。首次运行时，按当前放大链路校准 `--amplitude`。服务将 `--servo-duration` 设为 0，使视觉闭环持续工作；MCU 复位后回到按键监听。

每次按键启动并持续保持一个目标。想从直线换成圆，先复位 MCU，等第二个终端重新出现 `BUTTON_SERVICE_IDLE`，再按 MID；换成“∞”则按 RIGHT。重新测试同一目标也按这个顺序操作。

图形进入保持时，可以在对应日志中看到 `PHASE_SERVO_LOCKED`、`CIRCLE_SERVO_LOCKED` 或 `INFINITY_SERVO_LOCKED`。结合屏幕上的形状和后续帧，观察是否保持稳定。

需要单独调试某一目标时，先停止按键服务，再运行：

```bash
python3 requirement5_target.py --serial /dev/ttyUSB0 --config vision_config_mf500.json --target line --servo-duration 15 --output-dir ./target_output
```

把 `line` 换成 `circle` 或 `infinity` 可切换目标。这里的 15 表示形状伺服运行时长。固件命令 `fauto` 对应早期本地 CCD 路线；UVC 流程使用上述 Python 入口。

## 接线速查

| 用途 | 引脚 / 连接 |
| --- | --- |
| 电输入测频 / PFD 反馈 | PA16 / PA27，经前端调理后接入 |
| DAC 窗口斜坡 | PA15 |
| DDS SCLK / SDIO0 / CS / UPDATE / RESET | PA8 / PA9 / PB3 / PB2 / PA7 |
| DDS 图形输出 / PFD 参考 | CH0 / CH1 |
| 直通 / DDS 继电器 | PA13，低电平直通、高电平 DDS |
| 外部蜂鸣器与 LED | PA1，高有效，经 S8050 驱动 |
| UART0 TX / RX | PA10 / PA11，115200-8-N-1，USB-TTL 交叉连接并接地 |
| UART2 TX / RX | PB17 / PB18 |
| RGB R / G / B | PA2 / PA3 / PA4 |
| LEFT / DOWN / RIGHT / UP / MID | PB6 / PB7 / PB8 / PB9 / PB14 |
| 编码器 A / B / 按下 | PB15 / PB16 / PA12 |
| SWDIO / SWCLK / 外部 40 MHz | PA19 / PA20 / PA6 |
| 早期 TSL1401 AO / CLK / SI | PA14 / PA17 / PA18 |

## 流程停住时，从哪里看

| 现象 | 先检查 |
| --- | --- |
| 复位后没有启动信息 | 目标芯片、PA6 时钟、UART0 接线与波特率 |
| 相机服务运行但没有可用画面 | 设备名、`camera_active` 中的实际画幅、单帧图像和 ROI |
| 服务反复输出 `BUTTON_SERVICE_SERIAL_WAIT` | 串口设备名、访问权限、是否仍有其他程序占用 |
| 有按键请求，图形迟迟没有收敛 | 轨迹提取、频率结果、幅度配置及输出接线，逐段查看终端输出 |

跑通后，把源码提交、工具版本、相机配置、频点与目标形状写进同一份实验记录，保存终端输出和屏幕图像。控制时间从按键开始计，图形达到目标后继续观察至少 5 s。已有记录和后续需要补充的测试见[工程演进与测试记录](ARCHIVE.md)。
