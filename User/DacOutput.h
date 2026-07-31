#ifndef DAC_OUTPUT_H
#define DAC_OUTPUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DAC_OUTPUT_RATE_HZ 1000000U
#define DAC_OUTPUT_MIDCODE   2048U
#define DAC_OUTPUT_MAX_SAMPLES 10000U

void DacOutput_init(void);
void DacOutput_DACIRQ(void);
void DacOutput_TimerIRQ(void);
void DacOutput_setCode(uint16_t code);
bool DacOutput_play(const uint16_t *samples, size_t count);
bool DacOutput_playRamp(
    size_t count, uint16_t minimumCode, uint16_t maximumCode);
bool DacOutput_playWindowedRamp(uint32_t windowUs, uint32_t frameUs,
    uint16_t minimumCode, uint16_t maximumCode, uint16_t idleCode);
bool DacOutput_playWindowedRampEdgeBands(
    uint32_t windowUs, uint32_t frameUs,
    uint16_t minimumCode, uint16_t maximumCode,
    uint16_t lowIdleCode, uint16_t highIdleCode,
    uint16_t idleBandCodes);
bool DacOutput_takeDDSRecoveryRequest(void);
uint32_t DacOutput_getDMARecoveryCount(void);
void DacOutput_stop(void);

#endif
