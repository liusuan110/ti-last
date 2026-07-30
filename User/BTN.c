#include "BTN.h"

BTNTick_t BTNTick = {0};

void BTN_init(void) {
    /*
     * Ignore switch levels during the first debounce interval after reset.
     * This prevents a power-up transient on an active-low key from being
     * interpreted as a real press and starting calibration automatically.
     */
    BTNTick.left = BTN_DEBOUNCE_TIME;
    BTNTick.down = BTN_DEBOUNCE_TIME;
    BTNTick.right = BTN_DEBOUNCE_TIME;
    BTNTick.up = BTN_DEBOUNCE_TIME;
    BTNTick.mid = BTN_DEBOUNCE_TIME;
}

void BTN_getData(BTNData_t* data) {
    if (BTN_LEFT_PRESS && !BTNTick.left) {
        BTNTick.left = BTN_DEBOUNCE_TIME;
        data->left = 1;
    }
    else {
        data->left = 0;
    }
    if (BTN_DOWN_PRESS && !BTNTick.down) {
        BTNTick.down = BTN_DEBOUNCE_TIME;
        data->down = 1;
    }
    else {
        data->down = 0;
    }
    if (BTN_RIGHT_PRESS && !BTNTick.right) {
        BTNTick.right = BTN_DEBOUNCE_TIME;
        data->right = 1;
    }
    else {
        data->right = 0;
    }
    if (BTN_UP_PRESS && !BTNTick.up) {
        BTNTick.up = BTN_DEBOUNCE_TIME;
        data->up = 1;
    }
    else {
        data->up = 0;
    }
    if (BTN_MID_PRESS && !BTNTick.mid) {
        BTNTick.mid = BTN_DEBOUNCE_TIME;
        data->mid = 1;
    }
    else {
        data->mid = 0;
    }
}

void BTN_tick(void) {
    // 按键消抖倒计时
    if (!BTN_LEFT_PRESS && BTNTick.left) BTNTick.left--;
    if (!BTN_DOWN_PRESS && BTNTick.down) BTNTick.down--;
    if (!BTN_RIGHT_PRESS && BTNTick.right) BTNTick.right--;
    if (!BTN_UP_PRESS && BTNTick.up) BTNTick.up--;
    if (!BTN_MID_PRESS && BTNTick.mid) BTNTick.mid--;
}
