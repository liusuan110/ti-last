#include "AdcCapture.h"

#include "ti_msp_dl_config.h"

#define ADC_CAPTURE_FIFO_WORDS (ADC_CAPTURE_SIZE / 2U)
#define ADC_TIMER_CLOCK_HZ      40000000U

#if (ADC_CAPTURE_SIZE % 2U) != 0U
#error "ADC FIFO DMA requires an even ADC_CAPTURE_SIZE"
#endif

#if defined(__GNUC__) || defined(__clang__)
#define ADC_CAPTURE_ALIGN4 __attribute__((aligned(4)))
#else
#define ADC_CAPTURE_ALIGN4
#endif

static uint16_t gADC0[ADC_CAPTURE_SIZE] ADC_CAPTURE_ALIGN4;
static uint16_t gADC1[ADC_CAPTURE_SIZE] ADC_CAPTURE_ALIGN4;
static volatile bool gADC0Done;
static volatile bool gADC1Done;
static volatile bool gBusy;
static volatile bool gReady;
static uint32_t gSampleRateHz = ADC_CAPTURE_DEFAULT_RATE_HZ;

static void stopHardware(void);
static void serviceCompletion(void);

void AdcCapture_init(void)
{
    stopHardware();
    gADC0Done = false;
    gADC1Done = false;
    gBusy = false;
    gReady = false;
    (void)AdcCapture_setSampleRate(ADC_CAPTURE_DEFAULT_RATE_HZ);

    NVIC_ClearPendingIRQ(ADC0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ADC1_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC1_INST_INT_IRQN);
}

uint32_t AdcCapture_setSampleRate(uint32_t requestedRateHz)
{
    uint32_t loadValue;

    if (requestedRateHz < ADC_CAPTURE_MIN_RATE_HZ) {
        requestedRateHz = ADC_CAPTURE_MIN_RATE_HZ;
    } else if (requestedRateHz > ADC_CAPTURE_MAX_RATE_HZ) {
        requestedRateHz = ADC_CAPTURE_MAX_RATE_HZ;
    }

    loadValue =
        ((ADC_TIMER_CLOCK_HZ + (requestedRateHz / 2U)) / requestedRateHz) - 1U;
    DL_TimerG_setLoadValue(ADC_TRIGGER_TIMER_INST, loadValue);
    gSampleRateHz = ADC_TIMER_CLOCK_HZ / (loadValue + 1U);
    return gSampleRateHz;
}

uint32_t AdcCapture_getSampleRate(void)
{
    return gSampleRateHz;
}

bool AdcCapture_start(void)
{
    if (gBusy) {
        return false;
    }

    stopHardware();
    gADC0Done = false;
    gADC1Done = false;
    gReady = false;

    DL_DMA_setSrcAddr(
        DMA, DMA_CH0_CHAN_ID, (uint32_t)DL_ADC12_getFIFOAddress(ADC0_INST));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&gADC0[0]);
    DL_DMA_setTransferSize(
        DMA, DMA_CH0_CHAN_ID, ADC_CAPTURE_FIFO_WORDS);

    DL_DMA_setSrcAddr(
        DMA, DMA_CH1_CHAN_ID, (uint32_t)DL_ADC12_getFIFOAddress(ADC1_INST));
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)&gADC1[0]);
    DL_DMA_setTransferSize(
        DMA, DMA_CH1_CHAN_ID, ADC_CAPTURE_FIFO_WORDS);

    DL_ADC12_clearInterruptStatus(ADC0_INST, DL_ADC12_INTERRUPT_DMA_DONE);
    DL_ADC12_clearInterruptStatus(ADC1_INST, DL_ADC12_INTERRUPT_DMA_DONE);
    NVIC_ClearPendingIRQ(ADC0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ADC1_INST_INT_IRQN);

    gBusy = true;
    DL_ADC12_enableConversions(ADC0_INST);
    DL_ADC12_enableConversions(ADC1_INST);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
    DL_TimerG_setTimerCount(ADC_TRIGGER_TIMER_INST, 0U);
    DL_TimerG_startCounter(ADC_TRIGGER_TIMER_INST);
    return true;
}

void AdcCapture_abort(void)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    stopHardware();
    gBusy = false;
    if (interruptState == 0U) {
        __enable_irq();
    }
}

bool AdcCapture_isBusy(void)
{
    return gBusy;
}

bool AdcCapture_isReady(void)
{
    return gReady;
}

void AdcCapture_clear(void)
{
    gReady = false;
}

const uint16_t *AdcCapture_getADC0(void)
{
    return gADC0;
}

const uint16_t *AdcCapture_getADC1(void)
{
    return gADC1;
}

void AdcCapture_ADC0IRQ(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC0_INST) ==
        DL_ADC12_IIDX_DMA_DONE) {
        gADC0Done = true;
        serviceCompletion();
    }
}

void AdcCapture_ADC1IRQ(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC1_INST) ==
        DL_ADC12_IIDX_DMA_DONE) {
        gADC1Done = true;
        serviceCompletion();
    }
}

static void serviceCompletion(void)
{
    if (!gBusy || !gADC0Done || !gADC1Done) {
        return;
    }

    stopHardware();
    gBusy = false;
    gReady = true;
}

static void stopHardware(void)
{
    DL_TimerG_stopCounter(ADC_TRIGGER_TIMER_INST);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
    DL_ADC12_disableConversions(ADC0_INST);
    DL_ADC12_disableConversions(ADC1_INST);
}
