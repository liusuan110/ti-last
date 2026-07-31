#include "Encoder.h"

#define ENC_PENDING_LIMIT 32

static volatile int gEncoderDelta;
static volatile uint8_t gSwitchEvent;
static uint8_t gSwitchStablePressed;
static uint8_t gSwitchCandidatePressed;
static uint8_t gSwitchDebounceMs;
static uint8_t gSwitchArmed;

void ENC_init(void)
{
    uint8_t pressed = ENC_SW_PRESS ? 1U : 0U;

    gEncoderDelta = 0;
    gSwitchEvent = 0U;
    gSwitchStablePressed = pressed;
    gSwitchCandidatePressed = pressed;
    gSwitchDebounceMs = 0U;
    gSwitchArmed = pressed ? 0U : 1U;
    NVIC_EnableIRQ(GPIO_ENC_INT_IRQN);
}

int ENC_getIncVal(void)
{
    uint32_t interruptState = __get_PRIMASK();
    int value;

    __disable_irq();
    value = gEncoderDelta;
    gEncoderDelta = 0;
    if (interruptState == 0U) {
        __enable_irq();
    }
    return value;
}

uint8_t ENC_getSW(void)
{
    uint32_t interruptState = __get_PRIMASK();
    uint8_t event;

    __disable_irq();
    event = gSwitchEvent;
    gSwitchEvent = 0U;
    if (interruptState == 0U) {
        __enable_irq();
    }
    return event;
}

void ENC_tick(void)
{
    uint8_t rawPressed = ENC_SW_PRESS ? 1U : 0U;

    if (rawPressed != gSwitchCandidatePressed) {
        gSwitchCandidatePressed = rawPressed;
        gSwitchDebounceMs = 0U;
    } else if (gSwitchDebounceMs < ENC_SW_DEBOUNCE_MS) {
        gSwitchDebounceMs++;
    }

    if ((gSwitchDebounceMs >= ENC_SW_DEBOUNCE_MS) &&
        (gSwitchStablePressed != gSwitchCandidatePressed)) {
        gSwitchStablePressed = gSwitchCandidatePressed;
        if (gSwitchStablePressed != 0U) {
            if (gSwitchArmed != 0U) {
                gSwitchEvent = 1U;
            }
        } else {
            gSwitchArmed = 1U;
        }
    }
}

void GROUP1_IRQCallback(void)
{
    if (DL_GPIO_getPendingInterrupt(GPIO_ENC_A_PORT) == GPIO_ENC_A_IIDX) {
        if (DL_GPIO_readPins(GPIO_ENC_B_PORT, GPIO_ENC_B_PIN)) {
            if (gEncoderDelta > -ENC_PENDING_LIMIT) {
                gEncoderDelta--;
            }
        } else if (gEncoderDelta < ENC_PENDING_LIMIT) {
            gEncoderDelta++;
        }
    }
}
