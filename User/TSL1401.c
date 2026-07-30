#include "TSL1401.h"

#include <stddef.h>

#include "AdcCapture.h"
#include "Tick.h"
#include "ti_msp_dl_config.h"

#define TSL1401_AO_IOMUX       IOMUX_PINCM36
#define TSL1401_CLK_IOMUX      IOMUX_PINCM39
#define TSL1401_SI_IOMUX       IOMUX_PINCM40
#define TSL1401_GPIO_PORT       GPIOA
#define TSL1401_CLK_PIN         DL_GPIO_PIN_17
#define TSL1401_SI_PIN          DL_GPIO_PIN_18
#define TSL1401_ADC_CHANNEL     DL_ADC12_INPUT_CHAN_12
#define TSL1401_ADC_TIMEOUT     200000U
#define TSL1401_EDGE_IGNORE     4U
#define TSL1401_MAX_MERGE_GAP   1U
#define TSL1401_MIN_CONTRAST    40U

static uint16_t gFilterFrames[3][TSL1401_PIXEL_COUNT];
static uint16_t gFilteredLine[TSL1401_PIXEL_COUNT];

static void clockDelay(void);
static bool adcRead(uint16_t *value);
static bool readLine(uint16_t *pixels);
static void enterADCMode(void);
static void restoreADCMode(void);
static uint16_t medianValues(uint16_t *values, uint8_t count);
static void lineRange(const uint16_t *pixels,
    uint16_t *minimum, uint16_t *maximum);

void TSL1401_init(void)
{
    DL_GPIO_initPeripheralAnalogFunction(TSL1401_AO_IOMUX);
    DL_GPIO_initDigitalOutput(TSL1401_CLK_IOMUX);
    DL_GPIO_initDigitalOutput(TSL1401_SI_IOMUX);
    DL_GPIO_clearPins(
        TSL1401_GPIO_PORT, TSL1401_CLK_PIN | TSL1401_SI_PIN);
    DL_GPIO_enableOutput(
        TSL1401_GPIO_PORT, TSL1401_CLK_PIN | TSL1401_SI_PIN);
}

bool TSL1401_capture(uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint32_t exposureMs)
{
    bool ok;

    if (pixels == NULL) {
        return false;
    }
    if (exposureMs < 1U) {
        exposureMs = 1U;
    } else if (exposureMs > 100U) {
        exposureMs = 100U;
    }

    AdcCapture_abort();
    enterADCMode();

    /*
     * The first scan starts a new integration interval and discards the
     * previously accumulated charge. The second scan returns the pixels
     * accumulated during exposureMs.
     */
    ok = readLine(NULL);
    if (ok) {
        Tick_delay(exposureMs);
        ok = readLine(pixels);
    }

    DL_GPIO_clearPins(
        TSL1401_GPIO_PORT, TSL1401_CLK_PIN | TSL1401_SI_PIN);
    restoreADCMode();
    return ok;
}

bool TSL1401_captureFiltered(uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint32_t exposureMs, uint8_t frameCount)
{
    uint16_t values[3];
    uint32_t pixel;
    uint8_t frame;

    if (pixels == NULL) {
        return false;
    }
    if (frameCount <= 1U) {
        return TSL1401_capture(pixels, exposureMs);
    }
    if (frameCount > 3U) {
        frameCount = 3U;
    }
    if ((frameCount & 1U) == 0U) {
        frameCount--;
    }

    for (frame = 0U; frame < frameCount; frame++) {
        if (!TSL1401_capture(gFilterFrames[frame], exposureMs)) {
            return false;
        }
    }

    for (pixel = 0U; pixel < TSL1401_PIXEL_COUNT; pixel++) {
        for (frame = 0U; frame < frameCount; frame++) {
            values[frame] = gFilterFrames[frame][pixel];
        }
        pixels[pixel] = medianValues(values, frameCount);
    }
    return true;
}

bool TSL1401_captureAuto(uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint32_t *selectedExposureMs)
{
    static const uint8_t exposureMs[] = {1U, 2U, 5U, 10U, 20U, 40U};
    uint32_t bestScore = 0U;
    uint32_t score;
    uint16_t minimum;
    uint16_t maximum;
    uint16_t span;
    uint8_t bestIndex = 0U;
    uint8_t index;

    if (pixels == NULL) {
        return false;
    }

    for (index = 0U; index < sizeof(exposureMs); index++) {
        if (!TSL1401_capture(gFilterFrames[0], exposureMs[index])) {
            return false;
        }
        lineRange(gFilterFrames[0], &minimum, &maximum);
        span = maximum - minimum;

        /*
         * Prefer strong contrast while heavily penalizing saturation and a
         * raised black level. The first well-exposed frame wins ties.
         */
        score = span;
        if (maximum > 4000U) {
            score /= 8U;
        } else if (maximum > 3900U) {
            score /= 2U;
        }
        if (minimum > 1800U) {
            score /= 4U;
        } else if (minimum > 1000U) {
            score /= 2U;
        }
        if (score > bestScore) {
            bestScore = score;
            bestIndex = index;
        }

        if ((maximum >= 2600U) && (maximum <= 3900U) &&
            (minimum < 1000U) && (span >= 800U)) {
            bestIndex = index;
            break;
        }
    }

    if (selectedExposureMs != NULL) {
        *selectedExposureMs = exposureMs[bestIndex];
    }
    return TSL1401_captureFiltered(
        pixels, exposureMs[bestIndex], 3U);
}

void TSL1401_analyze(const uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint8_t thresholdPercent, TSL1401_Stats *stats)
{
    uint16_t minimum = 4095U;
    uint16_t maximum = 0U;
    uint16_t threshold;
    uint16_t *filtered = gFilteredLine;
    uint32_t i;
    uint32_t start;
    uint32_t end;

    if ((pixels == NULL) || (stats == NULL)) {
        return;
    }
    if (thresholdPercent < 10U) {
        thresholdPercent = 10U;
    } else if (thresholdPercent > 90U) {
        thresholdPercent = 90U;
    }

    filtered[0] = pixels[0];
    filtered[TSL1401_PIXEL_COUNT - 1U] =
        pixels[TSL1401_PIXEL_COUNT - 1U];
    for (i = 1U; i < (TSL1401_PIXEL_COUNT - 1U); i++) {
        filtered[i] = (uint16_t)(((uint32_t)pixels[i - 1U] +
            ((uint32_t)pixels[i] * 2U) + pixels[i + 1U]) / 4U);
    }

    for (i = TSL1401_EDGE_IGNORE;
         i < (TSL1401_PIXEL_COUNT - TSL1401_EDGE_IGNORE); i++) {
        if (filtered[i] < minimum) {
            minimum = filtered[i];
        }
        if (filtered[i] > maximum) {
            maximum = filtered[i];
        }
    }

    threshold = (uint16_t)(minimum +
        (((uint32_t)(maximum - minimum) * thresholdPercent) / 100U));
    stats->minimum = minimum;
    stats->maximum = maximum;
    stats->threshold = threshold;
    stats->peakCount = 0U;
    if ((maximum - minimum) < TSL1401_MIN_CONTRAST) {
        return;
    }

    i = TSL1401_EDGE_IGNORE;
    while (i < (TSL1401_PIXEL_COUNT - TSL1401_EDGE_IGNORE)) {
        uint32_t gap;

        if (filtered[i] < threshold) {
            i++;
            continue;
        }

        start = i;
        end = i;
        i++;
        while (i < (TSL1401_PIXEL_COUNT - TSL1401_EDGE_IGNORE)) {
            if (filtered[i] >= threshold) {
                end = i;
                i++;
                continue;
            }

            gap = 0U;
            while (((i + gap) <
                       (TSL1401_PIXEL_COUNT - TSL1401_EDGE_IGNORE)) &&
                   (filtered[i + gap] < threshold) &&
                   (gap <= TSL1401_MAX_MERGE_GAP)) {
                gap++;
            }
            if ((gap <= TSL1401_MAX_MERGE_GAP) &&
                ((i + gap) <
                    (TSL1401_PIXEL_COUNT - TSL1401_EDGE_IGNORE)) &&
                (filtered[i + gap] >= threshold)) {
                i += gap;
                continue;
            }
            break;
        }

        if (stats->peakCount < TSL1401_MAX_PEAKS) {
            uint8_t peak = stats->peakCount;
            stats->peakCenter[peak] = (uint8_t)((start + end) / 2U);
            stats->peakWidth[peak] = (uint8_t)(end - start + 1U);
            stats->peakCount++;
        }
    }
}

void TSL1401_subtractBackground(
    const uint16_t signal[TSL1401_PIXEL_COUNT],
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint16_t corrected[TSL1401_PIXEL_COUNT])
{
    uint32_t i;

    if ((signal == NULL) || (background == NULL) || (corrected == NULL)) {
        return;
    }
    for (i = 0U; i < TSL1401_PIXEL_COUNT; i++) {
        if (signal[i] > background[i]) {
            corrected[i] = signal[i] - background[i];
        } else {
            corrected[i] = 0U;
        }
    }
}

static void clockDelay(void)
{
    volatile uint32_t count;

    for (count = 0U; count < 20U; count++) {
        __NOP();
    }
}

static bool adcRead(uint16_t *value)
{
    uint32_t timeout = TSL1401_ADC_TIMEOUT;

    DL_ADC12_clearInterruptStatus(
        ADC0_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_startConversion(ADC0_INST);
    while ((DL_ADC12_getRawInterruptStatus(ADC0_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) &&
           (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        return false;
    }

    if (value != NULL) {
        *value = DL_ADC12_getMemResult(ADC0_INST, DL_ADC12_MEM_IDX_0);
    } else {
        (void)DL_ADC12_getMemResult(ADC0_INST, DL_ADC12_MEM_IDX_0);
    }
    DL_ADC12_enableConversions(ADC0_INST);
    return true;
}

static bool readLine(uint16_t *pixels)
{
    uint32_t i;
    uint16_t discard;

    DL_GPIO_clearPins(TSL1401_GPIO_PORT, TSL1401_CLK_PIN);
    DL_GPIO_setPins(TSL1401_GPIO_PORT, TSL1401_SI_PIN);
    clockDelay();
    DL_GPIO_setPins(TSL1401_GPIO_PORT, TSL1401_CLK_PIN);
    clockDelay();
    DL_GPIO_clearPins(TSL1401_GPIO_PORT, TSL1401_SI_PIN);
    clockDelay();

    for (i = 0U; i < TSL1401_PIXEL_COUNT; i++) {
        DL_GPIO_clearPins(TSL1401_GPIO_PORT, TSL1401_CLK_PIN);
        clockDelay();
        if (!adcRead((pixels != NULL) ? &pixels[i] : &discard)) {
            return false;
        }
        DL_GPIO_setPins(TSL1401_GPIO_PORT, TSL1401_CLK_PIN);
        clockDelay();
    }

    DL_GPIO_clearPins(TSL1401_GPIO_PORT, TSL1401_CLK_PIN);
    return true;
}

static void enterADCMode(void)
{
    static const DL_ADC12_ClockConfig clockConfig = {
        .clockSel = DL_ADC12_CLOCK_HFCLK,
        .divideRatio = DL_ADC12_CLOCK_DIVIDE_1,
        .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_32_TO_40,
    };

    NVIC_DisableIRQ(ADC0_INST_INT_IRQN);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_ADC12_disableConversions(ADC0_INST);
    DL_ADC12_disableInterrupt(ADC0_INST, 0xFFFFFFFFU);
    DL_ADC12_disableDMA(ADC0_INST);
    DL_ADC12_disableFIFO(ADC0_INST);

    DL_ADC12_setClockConfig(
        ADC0_INST, (DL_ADC12_ClockConfig *)&clockConfig);
    DL_ADC12_initSingleSample(ADC0_INST, DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC0_INST, DL_ADC12_MEM_IDX_0,
        TSL1401_ADC_CHANNEL, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(
        ADC0_INST, DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC0_INST, 40U);
    DL_ADC12_clearInterruptStatus(ADC0_INST, 0xFFFFFFFFU);
    DL_ADC12_enableConversions(ADC0_INST);
}

static void restoreADCMode(void)
{
    DL_ADC12_disableConversions(ADC0_INST);
    DL_ADC12_clearInterruptStatus(ADC0_INST, 0xFFFFFFFFU);
    SYSCFG_DL_ADC0_init();
    DL_ADC12_disableConversions(ADC0_INST);
    NVIC_ClearPendingIRQ(ADC0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC0_INST_INT_IRQN);
}

static uint16_t medianValues(uint16_t *values, uint8_t count)
{
    uint8_t i;
    uint8_t j;

    for (i = 1U; i < count; i++) {
        uint16_t value = values[i];
        j = i;
        while ((j > 0U) && (values[j - 1U] > value)) {
            values[j] = values[j - 1U];
            j--;
        }
        values[j] = value;
    }
    return values[count / 2U];
}

static void lineRange(const uint16_t *pixels,
    uint16_t *minimum, uint16_t *maximum)
{
    uint16_t low = 4095U;
    uint16_t high = 0U;
    uint32_t i;

    for (i = TSL1401_EDGE_IGNORE;
         i < (TSL1401_PIXEL_COUNT - TSL1401_EDGE_IGNORE); i++) {
        if (pixels[i] < low) {
            low = pixels[i];
        }
        if (pixels[i] > high) {
            high = pixels[i];
        }
    }
    *minimum = low;
    *maximum = high;
}
