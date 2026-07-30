# 2026 F题 MCU与DDS联调说明

## 当前已实现

- AD9959 单通道点频输出。
- 同频、正交、二倍频三种手动输出模式。
- 2/4/6/8 div 四档目标幅值接口。
- AD9959 四通道同时输出，用于DK-2500视觉并行探测。
- 左/中/右三个独立自动模式按键。
- DK-2500 ASCII串口协议、心跳和3秒失联保护。
- 自动模式状态灯：
  - 蓝色：等待DK-2500。
  - 青色：视觉搜索。
  - 黄色：频率/相位锁定。
  - 绿色：稳定完成。
  - 红色：错误或DK-2500失联。

## 按键定义

| 按键 | 功能 |
|---|---|
| LEFT | 一键自动直线 |
| MID | 一键自动圆 |
| RIGHT | 一键自动无穷图形 |
| UP | 手动模式轮换：同频 -> 正交 -> 二倍频 |
| DOWN | 幅值档位轮换：2 -> 4 -> 6 -> 8 div |
| 编码器旋转 | 手动调整AD9959相位字，每格64 |
| 编码器按下 | 关闭DDS输出 |

## 串口

- 当前控制/调试串口：UART0，PA10 TX、PA11 RX，115200 8N1。
- DK-2500通过USB-TTL连接UART0。
- 必须交叉连接TX/RX，并连接GND。
- 所有命令以换行结束。

## 常用命令

```text
fhelp
fstatus
fping

ffreq 10000
fdiv 8
fmode same
fmode quad
fmode double
fphase 4096
fmode off

fset 10000 512 0
fprobe 10000 10100 10200 10300 220

fauto line
fauto circle
fauto infinity
fstate search
fstate lock
fstate stable
fstate error
```

## DK-2500自动模式时序

1. 用户按LEFT、MID或RIGHT。
2. MCU关闭DDS并发送：

```text
F_EVENT auto=line request=1 freq_min=1000 freq_max=100000 step=100
```

3. DK-2500每3秒内至少发送一次`fping`或其他F命令。
4. 搜索阶段使用：

```text
fprobe F0 F1 F2 F3 AMP
```

5. 找到频率后使用：

```text
fset OUTPUT_FREQ AMP PHASE
fstate lock
```

6. 图形达到要求并确认稳定后发送：

```text
fstate stable
```

7. MCU保持输出并将RGB灯置为绿色。

## 必须等待硬件后完成的部分

### 输入测频和锁相

当前`ffreq`是人工写入的输入频率，用于先验证DDS输出。最终需要增加：

- 输入衰减和1.65V偏置。
- 比较器过零输出。
- MSPM0定时器捕获。
- 输入/输出相位误差计算。
- 同频和二倍频数字锁相环。

### 幅值标定

当前ASF只是线性占位映射：

| 目标跨度 | ASF |
|---:|---:|
| 2 div | 256 |
| 4 div | 512 |
| 6 div | 768 |
| 8 div | 1023 |

接上AD9959后级放大器后，必须使用MATLAB和示波器建立
`频率-目标跨度-ASF`标定表。

### 直通继电器

PA13已经配置为继电器控制输出，采用高电平吸合：

- PA13=0：继电器释放，NC到COM，信号源输入直通到装置输出。
- PA13=1：继电器吸合，NO到COM，DDS放大器连接到装置输出。
- 上电、`fmode off`、`fmode thru`和自动模式失联时默认回到直通。
- DDS模式会先关闭DDS、切换继电器并等待20ms，再打开DDS输出。

### 要求4幅度标定

2026-07-29在示波器X、Y轴均为0.5V/div、输入为10kHz且放大器接入的条件下，
用示波器Vpp测量完成了四档标定：

| Y轴目标 | 同频ASF（10kHz输出） | 二倍频ASF（20kHz输出） |
| --- | ---: | ---: |
| 2div / 1.0Vpp | 116 | 112 |
| 4div / 2.0Vpp | 244 | 240 |
| 6div / 3.0Vpp | 364 | 364 |
| 8div / 4.0Vpp | 494 | 489 |

10kHz、50kHz和100kHz输入抽测后，将6div两种模式统一留裕量为ASF=364；
全频段抽测误差均不超过0.12div。锁相输出工作时发送`fdiv 2|4|6|8`，
或者按一次DOWN键循环档位，固件会保持锁相和继电器状态，只更新CH0幅度。
这些数值是10kHz输入下的标定结果，正式全频段版本还需要加入频率补偿表。

### 四通道求和

`fprobe`已经同时配置AD9959四个通道，但必须把CH0至CH3通过高速加法器
合成为示波器Y通道信号，视觉并行探测才能生效。

## 首次上板测试顺序

1. 暂时只接AD9959 CH0到示波器CH1。
2. 上电后发送`fstatus`。
3. 发送`ffreq 10000`、`fdiv 8`、`fmode same`，检查10kHz输出。
4. 发送`fmode quad`，确认频率不变。
5. 发送`fmode double`，检查20kHz输出。
6. 依次发送`fdiv 2/4/6/8`，检查ASF和幅值单调变化。
7. 分别观察AD9959四个物理通道，再发送
   `fprobe 10000 10100 10200 10300 220`。
8. 按LEFT/MID/RIGHT，确认对应`F_EVENT`和RGB状态。
9. 自动模式下不发送心跳，确认3秒后DDS关闭且RGB变红。
