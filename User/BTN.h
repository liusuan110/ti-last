#ifndef BTN_H
#define BTN_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

#define BTN_LEFT_PRESS  (DL_GPIO_readPins(GPIO_BTN_PORT, GPIO_BTN_LEFT_PIN) == 0U)
#define BTN_DOWN_PRESS  (DL_GPIO_readPins(GPIO_BTN_PORT, GPIO_BTN_DOWN_PIN) == 0U)
#define BTN_RIGHT_PRESS (DL_GPIO_readPins(GPIO_BTN_PORT, GPIO_BTN_RIGHT_PIN) == 0U)
#define BTN_UP_PRESS    (DL_GPIO_readPins(GPIO_BTN_PORT, GPIO_BTN_UP_PIN) == 0U)
#define BTN_MID_PRESS   (DL_GPIO_readPins(GPIO_BTN_PORT, GPIO_BTN_MID_PIN) == 0U)

/* True stable-time debounce, sampled by the 1 ms SysTick callback. */
#define BTN_DEBOUNCE_MS  25U
#define BTN_LONG_PRESS_MS 700U

typedef struct BTNData_t {
    uint8_t left;
    uint8_t down;
    uint8_t right;
    uint8_t up;
    uint8_t mid;
    uint8_t leftLong;
    uint8_t downLong;
    uint8_t rightLong;
    uint8_t upLong;
    uint8_t midLong;
} BTNData_t;

void BTN_init(void);
void BTN_getData(BTNData_t *data);
void BTN_tick(void);

#endif
