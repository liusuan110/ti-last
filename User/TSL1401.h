#ifndef TSL1401_H
#define TSL1401_H

#include <stdbool.h>
#include <stdint.h>

#define TSL1401_PIXEL_COUNT 128U
#define TSL1401_MAX_PEAKS   24U

typedef struct TSL1401_Stats {
    uint16_t minimum;
    uint16_t maximum;
    uint16_t threshold;
    uint8_t peakCount;
    uint8_t peakCenter[TSL1401_MAX_PEAKS];
    uint8_t peakWidth[TSL1401_MAX_PEAKS];
} TSL1401_Stats;

/*
 * Wiring for the MSPM0G3507 48-pin device:
 *   TSL1401 AO  -> PA14 / ADC0 channel 12
 *   TSL1401 CLK -> PA17
 *   TSL1401 SI  -> PA18
 *   TSL1401 VCC -> 3.3 V
 *   TSL1401 GND -> GND
 *
 * PA27 remains dedicated to the phase-detector ADC input.
 */
void TSL1401_init(void);

/*
 * Capture one 128-pixel line. The phase detector must be stopped first
 * because this routine temporarily changes ADC0 from PA27 to PA14.
 * ADC0 is restored to its original PA27/DMA configuration before returning.
 */
bool TSL1401_capture(uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint32_t exposureMs);

/*
 * Median-combine 1 or 3 frames. This suppresses display refresh flicker
 * and isolated ADC noise without blurring the spatial positions of crossings.
 */
bool TSL1401_captureFiltered(uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint32_t exposureMs, uint8_t frameCount);

/*
 * Select an exposure from 1, 2, 5, 10, 20, or 40 ms, then return a
 * three-frame median line. The selected exposure is returned to the caller.
 */
bool TSL1401_captureAuto(uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint32_t *selectedExposureMs);

/*
 * Locate bright clusters in one line. thresholdPercent is measured from the
 * line minimum to maximum and is normally in the 35..70 percent range.
 */
void TSL1401_analyze(const uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint8_t thresholdPercent, TSL1401_Stats *stats);

/* Clamp-subtract a same-exposure background line (grid and fixed marks). */
void TSL1401_subtractBackground(
    const uint16_t signal[TSL1401_PIXEL_COUNT],
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint16_t corrected[TSL1401_PIXEL_COUNT]);

#endif
