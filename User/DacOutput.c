#include "DacOutput.h"

#include "AD9959.h"
#include "ti_msp_dl_config.h"

static uint16_t gRampSamples[DAC_OUTPUT_MAX_SAMPLES];
static const uint16_t * volatile gActiveSamples;
static volatile size_t gActiveCount;
static volatile bool gPlaying;
static volatile bool gDDSRecoveryRequired;
static volatile uint32_t gDMARecoveryCount;

static void armDMA(const uint16_t *samples, size_t count)
{
    /*
     * DMA channel 2 is a FULL channel on MSPM0G3507. Program repeat mode
     * explicitly every time so playback does not depend on stale peripheral
     * state left by an earlier ADC/DAC diagnostic.
     */
    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    DL_DMA_configTransfer(DMA, DMA_CH2_CHAN_ID,
        DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
        DL_DMA_NORMAL_MODE, DL_DMA_WIDTH_HALF_WORD,
        DL_DMA_WIDTH_HALF_WORD, DL_DMA_ADDR_INCREMENT,
        DL_DMA_ADDR_UNCHANGED);
    DL_DMA_setTrigger(DMA, DMA_CH2_CHAN_ID,
        DAC12_INST_DMA_TRIGGER, DL_DMA_TRIGGER_TYPE_EXTERNAL);
    DL_DMA_setSrcAddr(
        DMA, DMA_CH2_CHAN_ID, (uint32_t)&samples[0]);
    DL_DMA_setDestAddr(
        DMA, DMA_CH2_CHAN_ID, (uint32_t)&(DAC0->DATA0));
    DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, (uint16_t)count);
    DL_DMA_enableChannel(DMA, DMA_CH2_CHAN_ID);
}

void DacOutput_init(void)
{
    gActiveSamples = NULL;
    gActiveCount = 0U;
    gPlaying = false;
    gDDSRecoveryRequired = false;
    gDMARecoveryCount = 0U;
    DacOutput_stop();
    /*
     * Repeat-single DMA normally wraps without raising DMA_DONE. Keep the DAC
     * interrupt armed as a FIFO-headroom recovery path in case the peripheral
     * reports that no repeated transfer is pending. Servicing that condition
     * here avoids the former 0..1 ms SysTick-dependent gap at a 10 ms frame
     * boundary.
     */
    NVIC_SetPriority(DAC12_INT_IRQN, 0U);
    NVIC_ClearPendingIRQ(DAC12_INT_IRQN);
    NVIC_EnableIRQ(DAC12_INT_IRQN);
}

void DacOutput_DACIRQ(void)
{
    bool transferDone;

    /*
     * The MSPM0 DAC can disable its DMA trigger when DMA reports that no more
     * data is pending. The FIFO still contains the final samples at that point,
     * so reload the repeating frame directly from the DAC interrupt before the
     * FIFO drains. In the normal repeat-single path this interrupt never fires.
     */
    transferDone = (DL_DAC12_getInterruptStatus(
        DAC0, DL_DAC12_INTERRUPT_DMA_DONE) != 0U);
    if (transferDone) {
        DL_DAC12_clearInterruptStatus(
            DAC0, DL_DAC12_INTERRUPT_DMA_DONE);
    }
    if (transferDone && gPlaying && (gActiveSamples != NULL) &&
        (gActiveCount != 0U)) {
        armDMA(gActiveSamples, gActiveCount);
        DL_DAC12_enableDMATrigger(DAC0);
        gDMARecoveryCount++;
    }
}

void DacOutput_setCode(uint16_t code)
{
    uint32_t interruptState;

    if (code > 4095U) {
        code = 4095U;
    }

    /*
     * Electrically isolate every MCU-to-AD9959 control line before PA15
     * changes state. Holding MASTER_RESET high still drives the DDS board
     * during a DAC disturbance and can keep a partially powered device
     * latched. High impedance prevents current injection through all five
     * digital inputs; the caller rebuilds the bus and DDS state afterwards.
     */
    AD9959_busHiZ();
    AD9959_delayMicros(10U);

    /*
     * The DAC interrupt owns the emergency DMA reload path. Publish the stopped
     * state before touching the peripheral so the ISR cannot re-arm an old
     * waveform while static output is being selected.
     */
    interruptState = __get_PRIMASK();
    __disable_irq();
    gPlaying = false;
    gActiveSamples = NULL;
    gActiveCount = 0U;
    gDDSRecoveryRequired = true;
    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    DL_DAC12_disableSampleTimeGenerator(DAC0);
    DL_DAC12_disableDMATrigger(DAC0);
    DL_DAC12_disableFIFO(DAC0);
    DL_DAC12_clearInterruptStatus(DAC0, 0xFFFFFFFFU);
    DL_DAC12_output12(DAC0, code);
    DL_DAC12_enableOutputPin(DAC0);
    if (interruptState == 0U) {
        __enable_irq();
    }
}

bool DacOutput_play(const uint16_t *samples, size_t count)
{
    uint32_t interruptState;

    if ((samples == NULL) || (count == 0U) || (count > 65535U)) {
        return false;
    }

    /*
     * Leave all five DDS control pins high impedance for the complete
     * DMA-driven DAC interval. The caller performs one controlled DDS
     * initialization after DAC stop.
     */
    AD9959_busHiZ();
    AD9959_delayMicros(10U);

    interruptState = __get_PRIMASK();
    __disable_irq();
    gPlaying = false;
    gActiveSamples = NULL;
    gActiveCount = 0U;
    gDDSRecoveryRequired = true;
    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    DL_DAC12_disableSampleTimeGenerator(DAC0);
    DL_DAC12_disableDMATrigger(DAC0);
    DL_DAC12_disableFIFO(DAC0);
    DL_DAC12_disable(DAC0);

    /*
     * Reset the DAC state machine and FIFO before every DMA waveform. Merely
     * restoring the enable bits can leave the sample timer enabled without
     * producing its first FIFO/DMA request after a static-output test.
     */
    DL_DAC12_reset(DAC0);
    DL_DAC12_enablePower(DAC0);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_DAC12_init();
    DL_DAC12_clearInterruptStatus(DAC0, 0xFFFFFFFFU);

    gActiveSamples = samples;
    gActiveCount = count;
    armDMA(samples, count);

    DL_DAC12_setSampleRate(DAC0, DL_DAC12_SAMPLES_PER_SECOND_1M);
    DL_DAC12_enableOutputPin(DAC0);
    DL_DAC12_enableFIFO(DAC0);
    DL_DAC12_enableDMATrigger(DAC0);
    gPlaying = true;
    DL_DAC12_enableSampleTimeGenerator(DAC0);
    if (interruptState == 0U) {
        __enable_irq();
    }
    return true;
}

bool DacOutput_playRamp(
    size_t count, uint16_t minimumCode, uint16_t maximumCode)
{
    size_t index;
    uint32_t span;

    if ((count < 2U) || (count > DAC_OUTPUT_MAX_SAMPLES) ||
        (minimumCode > maximumCode) || (maximumCode > 4095U)) {
        return false;
    }

    span = (uint32_t)maximumCode - (uint32_t)minimumCode;
    for (index = 0U; index < count; index++) {
        gRampSamples[index] = (uint16_t)(
            (uint32_t)minimumCode +
            (span * (uint32_t)index) / (uint32_t)(count - 1U));
    }

    return DacOutput_play(gRampSamples, count);
}

bool DacOutput_playWindowedRamp(uint32_t windowUs, uint32_t frameUs,
    uint16_t minimumCode, uint16_t maximumCode, uint16_t idleCode)
{
    size_t frameSamples;
    size_t windowSamples;
    size_t index;
    uint32_t span;

    if ((windowUs < 2U) || (windowUs > frameUs) ||
        (minimumCode > maximumCode) || (maximumCode > 4095U) ||
        (idleCode > 4095U)) {
        return false;
    }

    frameSamples = (size_t)(
        ((uint64_t)frameUs * DAC_OUTPUT_RATE_HZ) / 1000000ULL);
    windowSamples = (size_t)(
        ((uint64_t)windowUs * DAC_OUTPUT_RATE_HZ) / 1000000ULL);
    if ((frameSamples < 2U) || (frameSamples > DAC_OUTPUT_MAX_SAMPLES) ||
        (windowSamples < 2U) || (windowSamples > frameSamples)) {
        return false;
    }

    /*
     * This is the MSPM0 equivalent of the reference design's 100 Hz phase
     * accumulator. At 1 MS/s a 10 ms frame is exactly 10000 samples, and the
     * repeat-single DMA wraps that table in hardware. The 1-2-5 window family
     * therefore maps directly to 10/5/2/1/0.5/0.2/0.1 ms without a foreground
     * restart at the frame boundary.
     */
    span = (uint32_t)maximumCode - (uint32_t)minimumCode;
    for (index = 0U; index < windowSamples; index++) {
        gRampSamples[index] = (uint16_t)(
            (uint32_t)minimumCode +
            (span * (uint32_t)index) / (uint32_t)(windowSamples - 1U));
    }
    for (; index < frameSamples; index++) {
        gRampSamples[index] = idleCode;
    }

    return DacOutput_play(gRampSamples, frameSamples);
}

bool DacOutput_playWindowedRampEdgeBands(
    uint32_t windowUs, uint32_t frameUs,
    uint16_t minimumCode, uint16_t maximumCode,
    uint16_t lowIdleCode, uint16_t highIdleCode,
    uint16_t idleBandCodes)
{
    size_t frameSamples;
    size_t windowSamples;
    size_t inactiveSamples;
    size_t highIdleSamples;
    size_t index;
    size_t step;
    uint32_t span;

    if ((windowUs < 2U) || (windowUs > frameUs) ||
        (idleBandCodes == 0U) ||
        (((uint32_t)lowIdleCode + idleBandCodes) > minimumCode) ||
        (minimumCode > maximumCode) ||
        (((uint32_t)maximumCode + idleBandCodes) > highIdleCode) ||
        (highIdleCode > 4095U)) {
        return false;
    }

    frameSamples = (size_t)(
        ((uint64_t)frameUs * DAC_OUTPUT_RATE_HZ) / 1000000ULL);
    windowSamples = (size_t)(
        ((uint64_t)windowUs * DAC_OUTPUT_RATE_HZ) / 1000000ULL);
    if ((frameSamples < 2U) || (frameSamples > DAC_OUTPUT_MAX_SAMPLES) ||
        (windowSamples < 2U) || (windowSamples > frameSamples)) {
        return false;
    }

    span = (uint32_t)maximumCode - (uint32_t)minimumCode;
    for (index = 0U; index < windowSamples; index++) {
        gRampSamples[index] = (uint16_t)(
            (uint32_t)minimumCode +
            (span * (uint32_t)index) / (uint32_t)(windowSamples - 1U));
    }

    /*
     * The active ramp ends one code below the high idle band and the next frame
     * starts one code above the low idle band. Distribute each half of the
     * inactive interval over every code in its edge band instead of dwelling
     * on one rail. This reduces the brightness of any individual horizontal
     * line by idleBandCodes while keeping paired high/low samples centred at
     * 2047.5. The sole large return remains an instantaneous, narrow vertical
     * line that vision can mask.
     */
    inactiveSamples = frameSamples - windowSamples;
    highIdleSamples = inactiveSamples / 2U;
    for (step = 0U; step < highIdleSamples; step++) {
        gRampSamples[index++] = (uint16_t)(
            (uint32_t)highIdleCode - idleBandCodes + 1U +
            ((uint32_t)step % idleBandCodes));
    }
    for (step = 0U; index < frameSamples; step++) {
        gRampSamples[index++] = (uint16_t)(
            (uint32_t)lowIdleCode + idleBandCodes - 1U -
            ((uint32_t)step % idleBandCodes));
    }

    return DacOutput_play(gRampSamples, frameSamples);
}

bool DacOutput_takeDDSRecoveryRequest(void)
{
    uint32_t interruptState = __get_PRIMASK();
    bool required;

    __disable_irq();
    required = gDDSRecoveryRequired;
    gDDSRecoveryRequired = false;
    if (interruptState == 0U) {
        __enable_irq();
    }
    return required;
}

uint32_t DacOutput_getDMARecoveryCount(void)
{
    return gDMARecoveryCount;
}

void DacOutput_stop(void)
{
    uint32_t interruptState = __get_PRIMASK();

    /*
     * Stop is a foreground/ISR hand-off point. Keep the state publication,
     * DMA shutdown and pending-flag clear atomic so the DAC interrupt cannot
     * resurrect the just-finished waveform while DDS GPIO writes are active.
     */
    __disable_irq();
    gPlaying = false;
    gActiveSamples = NULL;
    gActiveCount = 0U;
    DL_DAC12_disableSampleTimeGenerator(DAC0);
    DL_DAC12_disableDMATrigger(DAC0);
    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    DL_DAC12_disableFIFO(DAC0);
    DL_DAC12_clearInterruptStatus(DAC0, 0xFFFFFFFFU);
    DL_DAC12_output12(DAC0, DAC_OUTPUT_MIDCODE);
    DL_DAC12_disableOutputPin(DAC0);
    if (interruptState == 0U) {
        __enable_irq();
    }
}

void DacOutput_TimerIRQ(void)
{
    /* Retained for the shared interrupt table; DAC playback no longer uses it. */
}
