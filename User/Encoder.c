#include "Encoder.h"

volatile int ENCVal = 0;

volatile uint8_t ENCSWTick = 0;

void ENC_init(void) {
    NVIC_EnableIRQ(GPIO_ENC_INT_IRQN);
}

int ENC_getIncVal(void) {
    int val = ENCVal;
    ENCVal = 0;
    return val;
}

uint8_t ENC_getSW(void) {
    if (ENC_SW_PRESS && !ENCSWTick) {
        ENCSWTick = ENC_SW_DEBOUNCE_TIME;
        return 1;
    }
    else {
        return 0;
    }
}

void ENC_tick(void) {
    // 旋转编码器按钮消抖倒计时
    if (!ENC_SW_PRESS && ENCSWTick) ENCSWTick--;
}

// GPIO中断服务函数
void GROUP1_IRQCallback(void) {
    if (DL_GPIO_getPendingInterrupt(GPIO_ENC_A_PORT) == GPIO_ENC_A_IIDX) {
        if (DL_GPIO_readPins(GPIO_ENC_B_PORT, GPIO_ENC_B_PIN)) {
            ENCVal--;
        }
        else {
            ENCVal++;
        }
    }
}
