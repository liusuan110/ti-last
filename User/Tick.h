#ifndef TICK_H
#define TICK_H

#include <stdint.h>

extern volatile uint32_t Tick;

uint32_t Tick_now(void);
uint32_t Tick_elapsed(uint32_t start);
void Tick_delay(uint32_t milliseconds);
void Tick_SysTickCallback(void);

#endif
