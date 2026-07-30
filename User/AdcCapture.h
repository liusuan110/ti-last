#ifndef ADC_CAPTURE_H
#define ADC_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#define ADC_CAPTURE_SIZE              1024U
#define ADC_CAPTURE_DEFAULT_RATE_HZ 100000U
#define ADC_CAPTURE_MIN_RATE_HZ       1000U
#define ADC_CAPTURE_MAX_RATE_HZ    1000000U

void AdcCapture_init(void);
uint32_t AdcCapture_setSampleRate(uint32_t requestedRateHz);
uint32_t AdcCapture_getSampleRate(void);
bool AdcCapture_start(void);
void AdcCapture_abort(void);
bool AdcCapture_isBusy(void);
bool AdcCapture_isReady(void);
void AdcCapture_clear(void);
const uint16_t *AdcCapture_getADC0(void);
const uint16_t *AdcCapture_getADC1(void);

void AdcCapture_ADC0IRQ(void);
void AdcCapture_ADC1IRQ(void);

#endif
