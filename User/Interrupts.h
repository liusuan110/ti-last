#ifndef INTERRUPTS_H
#define INTERRUPTS_H

void SysTick_Handler(void);
void DAC0_IRQHandler(void);
void ADC0_IRQHandler(void);
void ADC1_IRQHandler(void);
void TIMG0_IRQHandler(void);
void GROUP1_IRQHandler(void);

#endif
