#include "Tick.h"

#include "BTN.h"
#include "Encoder.h"
#include "RGBLED.h"

volatile uint32_t Tick = 0U;

uint32_t Tick_now(void)
{
    return Tick;
}

uint32_t Tick_elapsed(uint32_t start)
{
    return (uint32_t)(Tick - start);
}

void Tick_delay(uint32_t milliseconds)
{
    uint32_t start = Tick_now();

    while (Tick_elapsed(start) < milliseconds) {
    }
}

void Tick_SysTickCallback(void)
{
    Tick++;
    RGBLED_RainbowTick++;
    BTN_tick();
    ENC_tick();
}
