#include "BTN.h"

#include <string.h>

enum {
    BTN_INDEX_LEFT = 0,
    BTN_INDEX_DOWN,
    BTN_INDEX_RIGHT,
    BTN_INDEX_UP,
    BTN_INDEX_MID,
    BTN_INDEX_COUNT
};

typedef struct BTNState {
    uint16_t holdMs;
    uint8_t debounceMs;
    uint8_t stablePressed;
    uint8_t candidatePressed;
    uint8_t armed;
    uint8_t longSent;
    volatile uint8_t pressEvent;
    volatile uint8_t longEvent;
} BTNState;

static BTNState gButton[BTN_INDEX_COUNT];

static uint8_t BTN_rawPressed(uint8_t index)
{
    uint32_t pins = DL_GPIO_readPins(GPIO_BTN_PORT,
        GPIO_BTN_LEFT_PIN | GPIO_BTN_DOWN_PIN | GPIO_BTN_RIGHT_PIN |
        GPIO_BTN_UP_PIN | GPIO_BTN_MID_PIN);
    uint32_t pin;

    switch (index) {
        case BTN_INDEX_LEFT:  pin = GPIO_BTN_LEFT_PIN; break;
        case BTN_INDEX_DOWN:  pin = GPIO_BTN_DOWN_PIN; break;
        case BTN_INDEX_RIGHT: pin = GPIO_BTN_RIGHT_PIN; break;
        case BTN_INDEX_UP:    pin = GPIO_BTN_UP_PIN; break;
        case BTN_INDEX_MID:
        default:              pin = GPIO_BTN_MID_PIN; break;
    }
    return ((pins & pin) == 0U) ? 1U : 0U;
}

static void BTN_setPress(BTNData_t *data, uint8_t index)
{
    switch (index) {
        case BTN_INDEX_LEFT:  data->left = 1U; break;
        case BTN_INDEX_DOWN:  data->down = 1U; break;
        case BTN_INDEX_RIGHT: data->right = 1U; break;
        case BTN_INDEX_UP:    data->up = 1U; break;
        case BTN_INDEX_MID:
        default:              data->mid = 1U; break;
    }
}

static void BTN_setLong(BTNData_t *data, uint8_t index)
{
    switch (index) {
        case BTN_INDEX_LEFT:  data->leftLong = 1U; break;
        case BTN_INDEX_DOWN:  data->downLong = 1U; break;
        case BTN_INDEX_RIGHT: data->rightLong = 1U; break;
        case BTN_INDEX_UP:    data->upLong = 1U; break;
        case BTN_INDEX_MID:
        default:              data->midLong = 1U; break;
    }
}

void BTN_init(void)
{
    uint8_t index;

    memset(gButton, 0, sizeof(gButton));
    for (index = 0U; index < BTN_INDEX_COUNT; index++) {
        uint8_t pressed = BTN_rawPressed(index);
        gButton[index].stablePressed = pressed;
        gButton[index].candidatePressed = pressed;
        /* A key held during reset must be released before it can trigger. */
        gButton[index].armed = pressed ? 0U : 1U;
    }
}

void BTN_getData(BTNData_t *data)
{
    uint32_t interruptState;
    uint8_t index;

    if (data == NULL) {
        return;
    }
    memset(data, 0, sizeof(*data));
    interruptState = __get_PRIMASK();
    __disable_irq();

    /* Deliver one press per main-loop call; simultaneous keys are queued. */
    for (index = 0U; index < BTN_INDEX_COUNT; index++) {
        if (gButton[index].pressEvent != 0U) {
            gButton[index].pressEvent = 0U;
            BTN_setPress(data, index);
            break;
        }
    }
    for (index = 0U; index < BTN_INDEX_COUNT; index++) {
        if (gButton[index].longEvent != 0U) {
            gButton[index].longEvent = 0U;
            BTN_setLong(data, index);
            break;
        }
    }
    if (interruptState == 0U) {
        __enable_irq();
    }
}

void BTN_tick(void)
{
    uint8_t index;

    for (index = 0U; index < BTN_INDEX_COUNT; index++) {
        BTNState *state = &gButton[index];
        uint8_t rawPressed = BTN_rawPressed(index);

        if (rawPressed != state->candidatePressed) {
            state->candidatePressed = rawPressed;
            state->debounceMs = 0U;
        } else if (state->debounceMs < BTN_DEBOUNCE_MS) {
            state->debounceMs++;
        }

        if ((state->debounceMs >= BTN_DEBOUNCE_MS) &&
            (state->stablePressed != state->candidatePressed)) {
            state->stablePressed = state->candidatePressed;
            state->holdMs = 0U;
            state->longSent = 0U;
            if (state->stablePressed != 0U) {
                if (state->armed != 0U) {
                    state->pressEvent = 1U;
                }
            } else {
                state->armed = 1U;
            }
        }

        if ((state->stablePressed != 0U) && (state->armed != 0U)) {
            if (state->holdMs < BTN_LONG_PRESS_MS) {
                state->holdMs++;
            }
            if ((state->holdMs >= BTN_LONG_PRESS_MS) &&
                (state->longSent == 0U)) {
                state->longSent = 1U;
                state->longEvent = 1U;
            }
        }
    }
}
