#include "Interrupts.h"

#include "AdcCapture.h"
#include "DacOutput.h"
#include "Encoder.h"
#include "Tick.h"

void SysTick_Handler(void)
{
    Tick_SysTickCallback();
    /*
     * Requirement 5 performs blocking CCD exposures. Keep the DAC DMA
     * waveform alive during those exposures instead of relying only on the
     * foreground App loop to re-arm a completed transfer.
     */
    DacOutput_service();
}

void ADC0_IRQHandler(void)
{
    AdcCapture_ADC0IRQ();
}

void ADC1_IRQHandler(void)
{
    AdcCapture_ADC1IRQ();
}

void TIMG0_IRQHandler(void)
{
    DacOutput_TimerIRQ();
}

void GROUP1_IRQHandler(void)
{
    GROUP1_IRQCallback();
}
