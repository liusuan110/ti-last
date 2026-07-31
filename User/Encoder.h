#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

#define ENC_SW_PRESS (DL_GPIO_readPins(GPIO_ENC_SW_PORT, GPIO_ENC_SW_PIN) == 0U)
#define ENC_SW_DEBOUNCE_MS 25U

void ENC_init(void);
int ENC_getIncVal(void);
uint8_t ENC_getSW(void);
void ENC_tick(void);
void GROUP1_IRQCallback(void);

#endif
