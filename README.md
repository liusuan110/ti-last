# 2026 电赛基础工程模板

这是从当前目录 4 个往年工程中提取出的通用 CCS 工程。目标是明天拿到题目后直接开始写赛题逻辑，而不是再次整理芯片、时钟、串口和基础驱动。

## 固定环境

- MCU：MSPM0G3507，LQFP-48
- IDE：Code Composer Studio
- 编译器：TI ARM LLVM 4.0.2 LTS
- SDK：MSPM0 SDK 2.10.00.04
- SysConfig：1.26.2
- CPU：80 MHz，外部 40 MHz HFCLK（PA6）
- 调度：裸机主循环 + 1 ms SysTick

模板上电后不会自动启动 ADC、DDS 或 DAC 波形播放。RGB 灯显示暗绿色，UART0 输出 `READY`。DAC0 保持中点码 2048；AD9959 总线只进入空闲状态，收到 `dds` 命令后才复位和初始化芯片。

## 明天主要修改的位置

1. 在 `User/App.c` 的 `App_loop()` 中加入赛题状态机。
2. 在 `User/App.h` 中放赛题对外接口。
3. 若引脚或外设需求改变，先改 `ContestTemplate_2026.syscfg`，不要手改 `Debug/syscfg` 生成文件。
4. 不需要的驱动可以从工程中删除；赛题算法建议新建独立的 `.c/.h`，不要全部堆进 `App.c`。

## 已准备的通用能力

| 模块 | 文件 | 默认能力 |
| --- | --- | --- |
| 应用入口 | `User/App.c/.h` | 初始化、主循环、串口命令和基础自检 |
| 1 ms 时基 | `User/Tick.c/.h` | 溢出安全的计时与延时 |
| 双 ADC DMA | `User/AdcCapture.c/.h` | ADC0/ADC1 同步触发，各 1024 点，1–200 kS/s |
| 片上 DAC | `User/DacOutput.c/.h` | PA15 直流码输出或 100 kS/s DMA 循环输出 |
| 外部 DDS | `User/AD9959.*`、`User/DDS.*` | AD9959 四通道点频、扫频和调制基础驱动 |
| 调试串口 | `User/UserUART.*` | UART0 行命令、非阻塞发送；UART2 基础发送 |
| 人机输入 | `User/BTN.*`、`User/Encoder.*` | 5 个按键与旋转编码器，含消抖 |
| 状态灯 | `User/RGBLED.*` | RGB 定色与彩虹显示 |
| 中断入口 | `User/Interrupts.*` | SysTick、双 ADC、编码器 GPIO |

CMSIS-DSP include 路径和 IQMath/MathACL 已写入工程，明天可直接增加 FFT、滤波或定点算法。

## 当前引脚表

| 功能 | 引脚 |
| --- | --- |
| ADC0 输入 | PA27 |
| ADC1 输入 | PA16 |
| DAC0 输出 | PA15 |
| AD9959 SCLK / SDIO0 / CS / UPDATE / RESET | PA8 / PA9 / PB3 / PB2 / PA7 |
| UART0 TX / RX（调试） | PA10 / PA11，115200 |
| UART2 TX / RX（屏幕） | PB17 / PB18，115200 |
| RGB R / G / B | PA2 / PA3 / PA4 |
| 五向按键 左 / 下 / 右 / 上 / 中 | PB6 / PB7 / PB8 / PB9 / PB14 |
| 编码器 A / B / SW | PB15 / PB16 / PA12 |
| SWDIO / SWCLK | PA19 / PA20 |
| 外部 40 MHz HFCLK | PA6 |

注意：PA6 的 40 MHz 外部时钟是当前 4 个旧工程共同采用的硬件前提。若明天换开发板或没有该时钟，必须先在 SysConfig 中重做时钟树。

## 串口自检命令

UART0 为 PA10(TX)/PA11(RX)，115200-8-N-1。命令以回车或换行结束：

```text
help
status
led 20 0 0
rainbow on
adc once 100000
dac 2048
dds 1000000 512 0
dds off
```

`adc once` 完成后输出两路 1024 点的最小值、最大值和平均值。`dds` 的幅度字范围为 0–1023，相位字范围为 0–16383。

## 导入与编译

在 CCS 中选择 **Import Project**，指向本目录 `ContestTemplate_2026`，然后执行 **Rebuild Project**。若 CCS 提示产品版本，选择本机的 MSPM0 SDK 2.10.00.04、SysConfig 1.26.2 和 TI ARM LLVM 4.0.2 LTS。

生成的 `Debug/` 不属于模板源文件，已经写入 `.gitignore`。烧录前务必核对芯片确为 MSPM0G3507，目标配置当前使用 SEGGER J-Link。

当前模板已用 TI ARM LLVM 4.0.2 LTS 通过 `-Wall -Wextra` 全量编译和链接：

- Flash：`0x5750`（22,352 B），剩余 108,720 B
- SRAM：`0x1556`（5,462 B），剩余 27,306 B
- 双 ADC 缓冲区已计入 SRAM 占用

## 设计约束

- 中断中只做采样完成标志、计数和输入捕获，算法放主循环。
- DMA 使用固定缓冲区时，缓冲区生命周期必须覆盖整个传输。
- `DacOutput_play()` 的输入数组在停止播放前不得释放或改写。
- ADC0/ADC1 当前都启用了 FIFO，DMA 以两个 16 位采样打包成一个 32 位字。
- 串口发送队列为 512 字节；高频遥测应二进制化、降采样或分帧，不能在快速循环中持续 `printf`。
- 变更 SysConfig 后重新全量编译，并重新核对生成的 `ti_msp_dl_config.h` 宏名。
