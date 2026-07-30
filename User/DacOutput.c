#include "DacOutput.h"

#include "ti_msp_dl_config.h"

static uint16_t gRampSamples[DAC_OUTPUT_MAX_SAMPLES];
static const uint16_t *gActiveSamples;
static size_t gActiveCount;
static bool gPlaying;

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
    DacOutput_stop();
}

void DacOutput_service(void)
{
    /*
     * Repeat mode should keep DMAEN set. Re-arm if a transient or an older
     * generated configuration clears it.
     */
    if (gPlaying && (gActiveSamples != NULL) && (gActiveCount != 0U) &&
        !DL_DMA_isChannelEnabled(DMA, DMA_CH2_CHAN_ID)) {
        armDMA(gActiveSamples, gActiveCount);
    }
}

void DacOutput_setCode(uint16_t code)
{
    if (code > 4095U) {
        code = 4095U;
    }

    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    DL_DAC12_disableSampleTimeGenerator(DAC0);
    DL_DAC12_disableDMATrigger(DAC0);
    DL_DAC12_disableFIFO(DAC0);
    DL_DAC12_output12(DAC0, code);
    DL_DAC12_enableOutputPin(DAC0);
    gActiveSamples = NULL;
    gActiveCount = 0U;
    gPlaying = false;
}

bool DacOutput_play(const uint16_t *samples, size_t count)
{
    if ((samples == NULL) || (count == 0U) || (count > 65535U)) {
        return false;
    }

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
    gPlaying = true;
    armDMA(samples, count);

    DL_DAC12_setSampleRate(DAC0, DL_DAC12_SAMPLES_PER_SECOND_1M);
    DL_DAC12_enableOutputPin(DAC0);
    DL_DAC12_enableFIFO(DAC0);
    DL_DAC12_enableDMATrigger(DAC0);
    DL_DAC12_enableSampleTimeGenerator(DAC0);
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

void DacOutput_stop(void)
{
    gPlaying = false;
    gActiveSamples = NULL;
    gActiveCount = 0U;
    DL_DAC12_disableSampleTimeGenerator(DAC0);
    DL_DAC12_disableDMATrigger(DAC0);
    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    DL_DAC12_disableFIFO(DAC0);
    DL_DAC12_output12(DAC0, DAC_OUTPUT_MIDCODE);
    DL_DAC12_disableOutputPin(DAC0);
}

void DacOutput_TimerIRQ(void)
{
    /* Retained for the shared interrupt table; DAC playback no longer uses it. */
}
