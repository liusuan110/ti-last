#include "AD9959.h"

#define AD9959_POWER_ON_SETTLE_US 100000U
#define AD9959_RESET_HIGH_US        1000U
#define AD9959_RESET_SETTLE_US     10000U
#define AD9959_REFCLK_SETTLE_US    10000U
#define AD9959_IO_UPDATE_PULSE_US     5U

/**
 * 引脚连接方式
 * System Configuration: GPIO组名: GPIO_DDS
 * MSPM0 GPIO引脚 -> AD9959模块引脚
 *
 * DDS_SCLK   -> AD9959_SCLK
 * DDS_SDIO0  -> AD9959_SDIO0
 * DDS_CS     -> AD9959_CS
 * DDS_UPDATE -> AD9959_UPDATE
 * DDS_RST    -> AD9959_RST
 * GND        -> GND
 *
 * AD9959模块其余数据引脚均接GND
*/

/* PLL disabled: use the stable 25 MHz reference clock directly. */
uint8_t FR1_DATA[3] = {0x00,0x00,0x00};

uint8_t FR2_DATA[2] = {0x00,0x00};    // 双方向扫描，即从起始值扫到结束值后，又从结束值扫到起始值
//uint8_t FR2_DATA[2] = {0x80,0x00};// 单方向扫描，即从起始值扫到结束值后，又从起始值扫到结束值，以此类推

float ACC_FRE_FACTOR = AD9959_FTW_PER_HZ;

/*
 * Normal single-tone channel state. A master reset leaves CFR[1]
 * (continuous phase-accumulator clear) set, so normal operation must write
 * that bit back to zero explicitly.
 */
uint8_t CFR_DATA[3] = {0x00,0x03,0x00};

/************************************************************
** 函数名称 ：void AD9959_delayMicros(uint32 us)
** 函数功能 ：微秒延时
** 入口参数 ：us: 需要延时的时间(μs)
** 函数说明 ：仅用于粗略延时
**************************************************************/
void AD9959_delayMicros(uint32_t us) {
    while (us--) {
        delay_cycles(80); // 对于80MHz系统时钟
    }
}

/************************************************************
** 函数名称 ：void AD9959_init(void)
** 函数功能 ：初始化控制AD9959需要用到的IO口,及寄存器
** 入口参数 ：无
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_init(void) {
    AD9959_IOInit(); // IO口电平状态初始化
    AD9959_RST_L;
    AD9959_delayMicros(AD9959_POWER_ON_SETTLE_US);
    AD9959_reset(); // AD9959复位

    /*
     * Keep the internal PLL/VCO disabled permanently. At 25 MHz the highest
     * required 200 kHz output still has 125 DDS samples per period. More
     * importantly, SYNC_CLK remains derived from the external reference and
     * cannot disappear because an internal x20 PLL lost lock.
     */
    AD9959_writeDataSlow(FR1_ADD, 3, FR1_DATA);
    AD9959_IOUpdate();
    AD9959_delayMicros(AD9959_REFCLK_SETTLE_US);
}

/************************************************************
** 函数名称 ：void AD9959_IOInit(void)
** 函数功能 ：IO口电平状态初始化
**************************************************************/
void AD9959_busHiZ(void) {
    uint32_t interruptState = __get_PRIMASK();

    /*
     * Do not drive an AD9959 that may still be unpowered. Driving CS, SCLK,
     * SDIO, UPDATE or RESET before DVDD_I/O is valid can forward-bias the
     * device's input protection diodes, partially power the chip and prevent
     * a clean POR. The resulting state can survive an MCU reset and may only
     * clear after the external DDS supply has discharged.
     */
    __disable_irq();
    DL_GPIO_disableOutput(GPIO_DDS_DDS_SCLK_PORT,
        GPIO_DDS_DDS_SCLK_PIN | GPIO_DDS_DDS_SDIO0_PIN |
        GPIO_DDS_DDS_RST_PIN);
    DL_GPIO_disableOutput(GPIO_DDS_DDS_CS_PORT, GPIO_DDS_DDS_CS_PIN);
    DL_GPIO_disableOutput(
        GPIO_DDS_DDS_UPDATE_PORT, GPIO_DDS_DDS_UPDATE_PIN);

    DL_GPIO_initDigitalInputFeatures(GPIO_DDS_DDS_SCLK_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_DDS_DDS_SDIO0_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_DDS_DDS_CS_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_DDS_DDS_UPDATE_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_DDS_DDS_RST_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    if (interruptState == 0U) {
        __enable_irq();
    }
}

void AD9959_IOInit(void) {
    uint32_t interruptState = __get_PRIMASK();

    /*
     * Re-apply the complete pin configuration, not only the output levels.
     * The pins remain high impedance from boot until this function is called
     * after the DDS supply has had time to become valid.
     */
    __disable_irq();
    DL_GPIO_initDigitalOutput(GPIO_DDS_DDS_SCLK_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_DDS_DDS_SDIO0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_DDS_DDS_CS_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_DDS_DDS_UPDATE_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_DDS_DDS_RST_IOMUX);

    DL_GPIO_clearPins(GPIO_DDS_DDS_SCLK_PORT,
        GPIO_DDS_DDS_SCLK_PIN | GPIO_DDS_DDS_SDIO0_PIN |
        GPIO_DDS_DDS_RST_PIN);
    DL_GPIO_setPins(GPIO_DDS_DDS_CS_PORT, GPIO_DDS_DDS_CS_PIN);
    DL_GPIO_clearPins(
        GPIO_DDS_DDS_UPDATE_PORT, GPIO_DDS_DDS_UPDATE_PIN);

    DL_GPIO_enableOutput(GPIO_DDS_DDS_SCLK_PORT,
        GPIO_DDS_DDS_SCLK_PIN | GPIO_DDS_DDS_SDIO0_PIN |
        GPIO_DDS_DDS_RST_PIN);
    DL_GPIO_enableOutput(GPIO_DDS_DDS_CS_PORT, GPIO_DDS_DDS_CS_PIN);
    DL_GPIO_enableOutput(
        GPIO_DDS_DDS_UPDATE_PORT, GPIO_DDS_DDS_UPDATE_PIN);

    AD9959_CS_H;
    AD9959_SCLK_L;
    AD9959_UPDATE_L;
    AD9959_SDIO0_L;
    if (interruptState == 0U) {
        __enable_irq();
    }
}

/************************************************************
** 函数名称 ：void AD9959_reset(void)
** 函数功能 ：AD9959复位
**************************************************************/
void AD9959_reset(void) {
    AD9959_RST_L;
    AD9959_delayMicros(100);
    AD9959_RST_H;
    AD9959_delayMicros(AD9959_RESET_HIGH_US);
    AD9959_RST_L;
    AD9959_delayMicros(AD9959_RESET_SETTLE_US);
}

void AD9959_holdReset(void) {
    uint32_t interruptState = __get_PRIMASK();

    /*
     * Keep the external DDS in master reset for the complete PA15 DAC
     * interval. This protects its PLL, serial controller and channel state
     * while the 1 MS/s DAC/DMA path is repeatedly restarted.
     */
    __disable_irq();
    DL_GPIO_initDigitalOutput(GPIO_DDS_DDS_RST_IOMUX);
    DL_GPIO_setPins(GPIO_DDS_DDS_RST_PORT, GPIO_DDS_DDS_RST_PIN);
    DL_GPIO_enableOutput(GPIO_DDS_DDS_RST_PORT, GPIO_DDS_DDS_RST_PIN);
    if (interruptState == 0U) {
        __enable_irq();
    }
    AD9959_delayMicros(10U);
}

/************************************************************
** 函数名称 void AD9959_IOUpdate(void)
** 函数功能 ： AD9959更新数据
**************************************************************/
void AD9959_IOUpdate(void) {
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    AD9959_UPDATE_L;
    AD9959_delayMicros(1U);
    AD9959_UPDATE_H;
    AD9959_delayMicros(AD9959_IO_UPDATE_PULSE_US);
    AD9959_UPDATE_L;
    AD9959_delayMicros(1U);
    if (interruptState == 0U) {
        __enable_irq();
    }
}

/************************************************************
** 函数名称 ：void AD9959_writeData(u8 regAddr, u8 regN, u8 *regData)
** 函数功能 ：使用模拟SPI向AD9959写数据
** 入口参数 ：regAddr: 寄存器地址
             regN: 要写入的字节数
             *regData: 数据起始地址
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeData(uint8_t regAddr, uint8_t regN, uint8_t* regData) {
    /*
     * Use the timing-qualified path for every register transaction. The
     * original zero-delay bit banging was marginal after the 1 MHz DAC/CCD
     * session and could leave the write-only DDS reporting success in MCU
     * state while CFTW/ACR/CPOW never reached the chip.
     */
    AD9959_writeDataSlow(regAddr, regN, regData);
}

/************************************************************
** 函数名称 ：void AD9959_writeDataSlow(u8 regAddr, u8 regN, u8 *regData)
** 函数功能 ：使用模拟SPI向AD9959慢速写数据
** 入口参数 ：regAddr: 寄存器地址
             regN: 要写入的字节数
             *regData: 数据起始地址
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeDataSlow(uint8_t regAddr, uint8_t regN, uint8_t* regData) {
    uint8_t ctrlVal = 0;
    uint8_t valToWrite = 0;
    uint8_t regIndex = 0;
    uint8_t i = 0;
    uint32_t interruptState = __get_PRIMASK();

    /*
     * A transaction must contain exactly 8 instruction clocks followed by
     * the register data clocks while CS remains low. Keep the short bit-bang
     * interval atomic so no unrelated interrupt can stretch it by milliseconds
     * or leave the write half-finished during a state transition.
     */
    __disable_irq();
    ctrlVal = regAddr;
    //写入地址
    AD9959_SCLK_L;
    AD9959_CS_L;
    AD9959_delayMicros(1);
    for (i = 0; i < 8; i++) {
        AD9959_SCLK_L;
        if (ctrlVal & 0x80) AD9959_SDIO0_H;
        else AD9959_SDIO0_L;
        AD9959_delayMicros(1);
        AD9959_SCLK_H;
        AD9959_delayMicros(1);
        ctrlVal <<= 1;
    }
    AD9959_SCLK_L;
    AD9959_delayMicros(1);
    //写入数据
    for (regIndex = 0; regIndex < regN; regIndex++) {
        valToWrite = regData[regIndex];
        for (i = 0; i < 8; i++) {
            AD9959_SCLK_L;
            if (valToWrite & 0x80) AD9959_SDIO0_H;
            else AD9959_SDIO0_L;
            AD9959_delayMicros(1);
            AD9959_SCLK_H;
            AD9959_delayMicros(1);
            valToWrite <<= 1;
        }
        AD9959_SCLK_L;
    }
    AD9959_CS_H;
    AD9959_delayMicros(1);
    if (interruptState == 0U) {
        __enable_irq();
    }
}

/************************************************************
** 函数名称 ：void AD9959_writeCFTW0(uint32_t freq)
** 函数功能 ：写CFTW0通道频率转换字寄存器
** 入口参数 ： freq:    写入频率，范围0~200 000 000 Hz
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeCFTW0(float freq) {
    uint32_t Temp;
    Temp = (uint32_t)(freq * ACC_FRE_FACTOR + 0.5F);
    AD9959_writeCFTW0Word(Temp);
}

void AD9959_writeCFTW0Word(uint32_t ftw) {
    uint8_t CFTW0_DATA[4] = {0x00,0x00,0x00,0x00};
    uint32_t Temp = ftw;
    CFTW0_DATA[3] = (uint8_t)Temp;
    CFTW0_DATA[2] = (uint8_t)(Temp >> 8);
    CFTW0_DATA[1] = (uint8_t)(Temp >> 16);
    CFTW0_DATA[0] = (uint8_t)(Temp >> 24);
    AD9959_writeData(CFTW0_ADD, 4, CFTW0_DATA);//CTW0 address 0x04
}

/************************************************************
** 函数名称 ：void AD9959_writeACR(uint16_t amp)
** 函数功能 ：写ACR通道幅度转换字寄存器
** 入口参数 ：Ampli:    输出幅度,范围0~1023，控制值0~1023对应输出幅度0~500mVpp左右
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeACR(uint16_t amp) {
    uint32_t A_temp = 0;
    uint8_t ACR_DATA[3] = {0x00,0x00,0x00};//default Value = 0x--0000 Rest = 18.91/Iout 
    A_temp = amp | 0x1000;

    ACR_DATA[1] = (uint8_t)(A_temp >> 8); //高位数据
    ACR_DATA[2] = (uint8_t)A_temp;  //低位数据
    AD9959_writeData(ACR_ADD, 3, ACR_DATA); //ACR address 0x06.CHn设定幅度
}

/************************************************************
** 函数名称 ：void AD9959_writeCPOW0(uint16_t phase)
** 函数功能 ：写CPOW0通道相位转换字寄存器
** 入口参数 ：Phase:        输出相位,范围：0~16383(对应角度：0°~360°)
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeCPOW0(uint16_t phase) {
    uint8_t CPOW0_data[2] = {0x00,0x00};
    CPOW0_data[1] = (uint8_t)phase;
    CPOW0_data[0] = (uint8_t)(phase >> 8);

    AD9959_writeData(CPOW0_ADD, 2, CPOW0_data);//CPOW0 address 0x05.CHn设定相位
}

/************************************************************
** 函数名称 ：void AD9959_writeLSRR(uint8_t rsrr,uint8_t fsrr)
** 函数功能 ：写LSRR线性扫描斜率寄存器
** 入口参数 ：    rsrr:    上升斜率,范围：0~255
                            fsrr:    下降斜率,范围：0~255
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeLSRR(uint8_t rsrr, uint8_t fsrr) {
    uint8_t LSRR_data[2] = {0x00,0x00};

    LSRR_data[1] = rsrr;
    LSRR_data[0] = fsrr;//高8位下降斜率

    AD9959_writeData(LSRR_ADD, 2, LSRR_data);
}

/************************************************************
** 函数名称 ：void AD9959_writeRDW(uint32_t r_delta)
** 函数功能 ：写RDW上升增量寄存器
** 入口参数 ：r_delta:上升增量,0-4294967295
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeRDW(uint32_t r_delta) {
    uint8_t RDW_data[4] = {0x00,0x00,0x00,0x00};    //中间变量

    RDW_data[3] = (uint8_t)r_delta;
    RDW_data[2] = (uint8_t)(r_delta >> 8);
    RDW_data[1] = (uint8_t)(r_delta >> 16);
    RDW_data[0] = (uint8_t)(r_delta >> 24);
    AD9959_writeData(RDW_ADD, 4, RDW_data);
}

/************************************************************
** 函数名称 ：void AD9959_writeFDW(uint32_t f_delta)
** 函数功能 ：写FDW下降增量寄存器
** 入口参数 ：f_delta:下降增量,0-4294967295
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeFDW(uint32_t f_delta) {
    uint8_t FDW_data[4] = {0x00,0x00,0x00,0x00};    //中间变量

    FDW_data[3] = (uint8_t)f_delta;
    FDW_data[2] = (uint8_t)(f_delta >> 8);
    FDW_data[1] = (uint8_t)(f_delta >> 16);
    FDW_data[0] = (uint8_t)(f_delta >> 24);
    AD9959_writeData(FDW_ADD, 4, FDW_data);
}

/************************************************************
** 函数名称 ：void AD9959_writeProfileFreq(uint8_t profile,uint32_t data)
** 函数功能 ：写Profile寄存器
** 入口参数 ：profile:    profile号(0~14)
                            data:    写入频率，范围0~200 000 000 Hz
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeProfileFreq(uint8_t profile, uint32_t data) {
    uint8_t profileAddr;
    uint8_t Profile_data[4] = {0x00,0x00,0x00,0x00};    //中间变量
    uint32_t Temp;
    Temp = (uint32_t)data * ACC_FRE_FACTOR;       //将输入频率因子分为四个字节  4.294967296=(2^32)/500000000
    Profile_data[3] = (uint8_t)Temp;
    Profile_data[2] = (uint8_t)(Temp >> 8);
    Profile_data[1] = (uint8_t)(Temp >> 16);
    Profile_data[0] = (uint8_t)(Temp >> 24);
    profileAddr = PROFILE_ADDR_BASE + profile;

    AD9959_writeData(profileAddr, 4, Profile_data);
}
/************************************************************
** 函数名称 ：void AD9959_writeProfileAmp(uint8_t profile,uint16_t data)
** 函数功能 ：写Profile寄存器
** 入口参数 ：profile:    profile号(0~14)
                            data:     写入幅度,范围0~1023，
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeProfileAmp(uint8_t profile, uint16_t data) {
    uint8_t profileAddr;
    uint8_t Profile_data[4] = {0x00,0x00,0x00,0x00};    //中间变量

    //为幅度调制时幅度数据为[31:22]位
    Profile_data[1] = (uint8_t)(data << 6);//[23:22]
    Profile_data[0] = (uint8_t)(data >> 2);//[31:24]

    profileAddr = PROFILE_ADDR_BASE + profile;

    AD9959_writeData(profileAddr, 4, Profile_data);//写入32位数据
}
/************************************************************
** 函数名称 ：void AD9959_writeProfilePhase(uint8_t profile,uint16_t data)
** 函数功能 ：写Profile寄存器
** 入口参数 ：profile:    profile号(0~14)
                            data:     写入相位,范围：0~16383
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_writeProfilePhase(uint8_t profile, uint16_t data) {
    uint8_t profileAddr;
    uint8_t Profile_data[4] = {0x00,0x00,0x00,0x00};    //中间变量

    //为相位调制时相位数据为[31:18]位
    Profile_data[1] = (uint8_t)(data << 2);//[23:18]
    Profile_data[0] = (uint8_t)(data >> 6);//[31:24]

    profileAddr = PROFILE_ADDR_BASE + profile;

    AD9959_writeData(profileAddr, 4, Profile_data);//写入32位数据
}



/************************************************************
** 函数名称 ：void AD9959_setFreq(uint8_t channel,uint32_t freq)
** 函数功能 ：设置通道的输出频率
** 入口参数 ：channel:  输出通道  AD9959_CH0-AD9959_CH3
                         freq:     输出频率，范围0~200 000 000 Hz
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_setFreq(uint8_t channel, float freq) {
    uint8_t CHANNEL[1] = {0x00};

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL);//控制寄存器写入CHn通道，选择CHn
    AD9959_writeCFTW0(freq);//输出CHn设定频率                                                                                                                                             
}

void AD9959_setFTW(uint8_t channel, uint32_t ftw) {
    uint8_t CHANNEL[1] = {channel};

    AD9959_writeData(CSR_ADD, 1, CHANNEL);
    AD9959_writeCFTW0Word(ftw);
}

/*
 * Pulse the FR2[12] continuous-clear control. The first I/O update holds all
 * phase accumulators at zero; the second clears FR2[12] and releases every
 * accumulator from the same system-clock state. This avoids relying on the
 * edge semantics of the FR2[13] autoclear control during repeated updates.
 */
void AD9959_syncPhaseAccumulators(void) {
    uint8_t clearPhase[2] = {0x10, 0x00};
    uint8_t normalRun[2] = {0x00, 0x00};

    AD9959_writeData(FR2_ADD, 2, clearPhase);
    AD9959_IOUpdate();
    AD9959_writeData(FR2_ADD, 2, normalRun);
    AD9959_IOUpdate();
}

void AD9959_configureSingleToneChannels(void) {
    uint8_t allChannels[1] = {
        AD9959_CH0 | AD9959_CH1 | AD9959_CH2 | AD9959_CH3
    };

    AD9959_writeDataSlow(CSR_ADD, 1, allChannels);
    AD9959_writeDataSlow(CFR_ADD, 3, CFR_DATA);
    AD9959_writeDataSlow(FR2_ADD, 2, FR2_DATA);
    AD9959_IOUpdate();
}

/************************************************************
** 函数名称 ：void AD9959_setAmp(uint8_t channel, uint16_t amp)
** 函数功能 ：设置通道的输出幅度
** 入口参数 ：channel:  输出通道 AD9959_CH0-AD9959_CH3
                            amp:    输出幅度,范围0~1023，控制值0~1023对应输出幅度0~500mVpp左右
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_setAmp(uint8_t channel, uint16_t amp) {
    uint8_t CHANNEL[1] = {0x00};

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn
    AD9959_writeACR(amp);                            //    CHn设定幅度
}

/************************************************************
** 函数名称 ：void AD9959_setPhase(uint8_t channel,uint16_t phase)
** 函数功能 ：设置通道的输出相位
** 入口参数 ：channel:  输出通道 AD9959_CH0-AD9959_CH3
                            phase:        输出相位,范围：0~16383(对应角度：0°~360°)
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_setPhase(uint8_t channel, uint16_t phase) {
    uint8_t CHANNEL[1] = {0x00};
    CHANNEL[0] = channel;

    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn
    AD9959_writeCPOW0(phase);//CHn设定相位
}

/************************************************************
** 函数名称 ：void AD9959_modulationInit(uint8_t channel,uint8_t modulation,uint8_t sweepEn,uint8_t nLevel)
** 函数功能 ：设置各个通道的调制模式。
** 入口参数 ： channel:      输出通道 AD9959_CH0-AD9959_CH3
                            modulation:    调制模式DISABLE_MOD，ASK，FSK，PSK
                            sweepEn:        线性扫描模式 SWEEP_ENABLE启用、SWEEP_DISABLE不启用；启用时Nlevel只能是LEVEL_MOD_2
                            nLevel：        调制电平选择 LEVEL_MOD_2、4、8、16
** 出口参数 ：无
** 函数说明 ：如将调制电平设置为2电平调制时，对应的P0-P3引脚上的高低电平分别控制CH0-CH3通道(如果对应通道开启的话)
                            如将调制电平设置为4电平调制时，对应的P0，P1和P2,P3引脚上的高低电平分别控制CH0-CH1通道(如果对应通道开启的话)
                            由于AD9959只有P0-P3,4个用于调制控制的引脚，因此输出在4电平调制时，只能有2个通道同时设置为调制输出；
                            8电平和16电平调制时，只能有1个通道同时设置为调制输出。请适当设置几电平调制以满足应用需求。

**注意！！！：设置成4电平调制时，输出通道只能选择CH0-1
                            设置成8,16电平调制时，输出通道只能选择CH0
                            本函数未做任意通道兼容，具体方法请参考AD9959芯片手册22-23页，操作FR1[14:12]为对应值。
**************************************************************/
void AD9959_modulationInit(uint8_t channel, uint8_t modulation, uint8_t sweepEn, uint8_t nLevel) {
    uint8_t i = 0;
    uint8_t CHANNEL[1] = {0x00};
    uint8_t FR1_data[3];
    uint8_t FR2_data[2];
    uint8_t CFR_data[3];
    for (i = 0;i < 3;i++)//设置默认值
    {
        FR1_data[i] = FR1_DATA[i];
        CFR_data[i] = CFR_DATA[i];
    }
    FR2_data[0] = FR2_DATA[0];
    FR2_data[1] = FR2_DATA[1];

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn；以下设置均针对CHn

    FR1_data[1] = nLevel;
    CFR_data[0] = modulation;
    CFR_data[1] |= sweepEn;
    CFR_data[2] = 0x00;

    if (channel != 0)//有通道被使能
    {
        AD9959_writeData(FR1_ADD, 3, FR1_data);//写功能寄存器1
        AD9959_writeData(FR2_ADD, 2, FR2_data);//写功能寄存器1
        AD9959_writeData(CFR_ADD, 3, CFR_data);//写通道功能寄存器
    }
}


/************************************************************
** 函数名称 ：void AD9959_setFSK(uint8_t channel, uint32_t *data,uint16_t phase)
** 函数功能 ：设置FSK调制的参数
** 入口参数 ：channel:  输出通道 AD9959_CH0-AD9959_CH3
                            *data:    调整频率数据的起始地址
                            phase:    输出相位,范围：0~16383(对应角度：0°~360°)
** 出口参数 ：无
** 函数说明 ：FSK时信号幅度默认为最大
**************************************************************/
void AD9959_setFSK(uint8_t channel, uint32_t* data, uint16_t phase) {
    uint8_t i = 0;
    uint8_t CHANNEL[1] = {0x00};

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn；以下设置均针对CHn
    AD9959_writeCPOW0(phase);//设置相位

    AD9959_writeCFTW0(data[0]);
    for (i = 0;i < 15;i++)
        AD9959_writeProfileFreq(i, data[i + 1]);
}

/************************************************************
** 函数名称 ：void AD9959_setASK(uint8_t channel, uint32_t *data,uint32_t freq,uint16_t phase)
** 函数功能 ：设置ASK调制的参数
** 入口参数 ：channel:  输出通道 AD9959_CH0-AD9959_CH3
                            *data: 调整幅度数据的起始地址
                            freq:        输出频率，范围0~200 000 000 Hz
                            phase:    输出相位,范围：0~16383(对应角度：0°~360°)
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_setASK(uint8_t channel, uint16_t* data, uint32_t freq, uint16_t phase) {
    uint8_t i = 0;
    uint8_t CHANNEL[1] = {0x00};

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn；以下设置均针对CHn
    AD9959_writeCFTW0(freq);//设置频率
    AD9959_writeCPOW0(phase);//设置相位

    AD9959_writeACR(data[0]);
    for (i = 0;i < 15;i++)
        AD9959_writeProfileAmp(i, data[i + 1]);
}

/************************************************************
** 函数名称 ：void AD9959_setPSK(uint8_t channel, uint16_t *data,uint32_t freq,uint16_t phase)
** 函数功能 ：设置PSK调制的参数
** 入口参数 ：channel:  输出通道 AD9959_CH0-AD9959_CH3
                            *data:    调整相位数据的起始地址
                            freq:        输出频率，范围0~200 000 000 Hz
** 出口参数 ：无
** 函数说明 ：无
**************************************************************/
void AD9959_setPSK(uint8_t channel, uint16_t* data, uint32_t freq) {
    uint8_t i = 0;
    uint8_t CHANNEL[1] = {0x00};

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn；以下设置均针对CHn
    AD9959_writeCFTW0(freq);

    AD9959_writeCPOW0(data[0]);
    for (i = 0;i < 15;i++)
        AD9959_writeProfilePhase(i, data[i + 1]);
}

/************************************************************
** 函数名称 ：void AD9959_setFreqSweep(uint8_t channel, uint32_t s_data,uint32_t e_data,uint8_t fsrr,uint8_t rsrr,uint32_t r_delta,uint32_t f_delta,uint16_t phase)
** 函数功能 ：设置线性扫频的参数
** 入口参数 ：channel:  输出通道 AD9959_CH0-AD9959_CH3
                            s_data:    起始频率，范围0~200 000 000 Hz
                            e_data:    结束频率，范围0~200 000 000 Hz
                            r_delta:上升步长频率,0~200 000 000Hz
                            f_delta:下降步长频率,0~200 000 000Hz

                            rsrr:        上升斜率,范围：1~255，系统时钟为500Mhz时一个控制字约为8ns
                            fsrr:        下降斜率,范围：1~255
                            amp:    输出幅度,范围0~1023，控制值0~1023对应输出幅度0~500mVpp左右
                            phase:    输出相位,范围：0~16383(对应角度：0°~360°)
** 出口参数 ：无
** 函数说明 ：频点与频点间停留时间 dT = Xsrr*8 单位ns，扫描点数=(起始-结束)/步长
                            扫频总时间=总扫描频点数*dT
**************************************************************/
void AD9959_setFreqSweep(uint8_t channel, uint32_t s_data, uint32_t e_data, uint32_t r_delta, uint32_t f_delta, uint8_t rsrr, uint8_t fsrr, uint16_t amp, uint16_t phase) {
    uint8_t CHANNEL[1] = {0x00};
    uint32_t Fer_data = 0;

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn；以下设置均针对CHn

    AD9959_writeCPOW0(phase);//设置相位
    AD9959_writeACR(amp); //幅度设置

    AD9959_writeLSRR(rsrr, fsrr);//斜率

    Fer_data = (uint32_t)r_delta * ACC_FRE_FACTOR;     //频率转换成控制字
    AD9959_writeRDW(Fer_data);//上升步长

    Fer_data = (uint32_t)f_delta * ACC_FRE_FACTOR;
    AD9959_writeFDW(Fer_data);//下降步长

    AD9959_writeCFTW0(s_data);//起始频率
    AD9959_writeProfileFreq(0, e_data);//结束频率
}

/************************************************************
** 函数名称 ：void AD9959_setAmpSweep(uint8_t channel, uint32_t s_Ampli,uint16_t e_Ampli,uint32_t r_delta,uint32_t f_delta,uint8_t rsrr,uint8_t fsrr,uint32_t freq,uint16_t phase)
** 函数功能 ：设置线性扫幅的参数
** 入口参数 ：channel:  输出通道 AD9959_CH0-AD9959_CH3
                            s_Ampli:    起始幅度，控制值0~1023对应输出幅度0~500mVpp左右
                            e_Ampli:    结束幅度，

                            r_delta:    上升步长幅度,0~1023
                            f_delta:    下降步长幅度,0~1023

                            rsrr:            上升斜率,范围：1~255，系统时钟为500Mhz时一个控制字约为8ns
                            fsrr:            下降斜率,范围：1~255

                            freq:            输出频率，范围0~200 000 000 Hz
                            phase:        输出相位,范围：0~16383(对应角度：0°~360°)
** 出口参数 ：无
** 函数说明 ：幅点与幅点间停留时间 dT = Xsrr*8 单位ns；扫描点数=(起始-结束)/步长
                            扫幅总时间=总扫描幅点数*dT
**************************************************************/
void AD9959_setAmpSweep(uint8_t channel, uint32_t s_Ampli, uint16_t e_Ampli, uint32_t r_delta, uint32_t f_delta, uint8_t rsrr, uint8_t fsrr, uint32_t freq, uint16_t phase) {
    uint8_t CHANNEL[1] = {0x00};
    uint8_t ACR_data[3] = {0x00,0x00,0x00};

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn；以下设置均针对CHn

    AD9959_writeCFTW0(freq); //幅度频率
    AD9959_writeCPOW0(phase);//设置相位

    AD9959_writeLSRR(rsrr, fsrr);//斜率

    AD9959_writeRDW(r_delta << 22);//上升步长

    AD9959_writeFDW(f_delta << 22);//下降步长

    ACR_data[1] = (uint8_t)(s_Ampli >> 8); //高位数据
    ACR_data[2] = (uint8_t)s_Ampli;  //低位数据
    AD9959_writeData(ACR_ADD, 3, ACR_data); //ACR 设定起始幅度

    AD9959_writeProfileAmp(0, e_Ampli);//结束幅度
}


/************************************************************
** 函数名称 ：void AD9959_setPhaseSweep(uint8_t channel, uint16_t s_data,uint16_t e_data,uint16_t r_delta,uint16_t f_delta,uint8_t rsrr,uint8_t fsrr,uint32_t freq,uint16_t amp)
** 函数功能 ：设置线性扫相的参数
** 入口参数 ：channel:  输出通道 AD9959_CH0-AD9959_CH3
                            s_data:    起始相位，范围：0~16383(对应角度：0°~360°)
                            e_data:    结束相位，
                            r_delta:上升步长,范围：0~16383(对应角度：0°~360°)
                            f_delta:下降步长,

                            rsrr:        上升斜率,范围：1~255，系统时钟为500Mhz时一个控制字约为8ns
                            fsrr:        下降斜率,范围：1~255
                            freq:        输出频率，范围0~200 000 000 Hz
                            amp:    输出幅度,范围0~1023，控制值0~1023对应输出幅度0~500mVpp左右
** 出口参数 ：无
** 函数说明 ：频点与频点间停留时间 dT = Xsrr*8 单位ns；扫描点数=(起始-结束)/步长
                            扫频总时间=总扫描频点数*dT
**************************************************************/
void AD9959_setPhaseSweep(uint8_t channel, uint16_t s_data, uint16_t e_data, uint16_t r_delta, uint16_t f_delta, uint8_t rsrr, uint8_t fsrr, uint32_t freq, uint16_t amp) {
    uint8_t CHANNEL[1] = {0x00};

    CHANNEL[0] = channel;
    AD9959_writeData(CSR_ADD, 1, CHANNEL); //控制寄存器写入CHn通道，选择CHn；以下设置均针对CHn

    AD9959_writeCFTW0(freq); //幅度频率
    AD9959_writeACR(amp); //幅度设置

    AD9959_writeLSRR(rsrr, fsrr);//斜率

    AD9959_writeRDW(r_delta << 18);//上升步长

    AD9959_writeFDW(f_delta << 18);//下降步长

    AD9959_writeCPOW0(s_data);//起始相位
    AD9959_writeProfilePhase(0, e_data);//结束相位
}
