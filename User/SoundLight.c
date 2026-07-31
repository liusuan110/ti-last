#include "SoundLight.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

/*
 * PA1 is reserved in SysConfig as GPIO_SOUND_LIGHT_CTRL. Keep these fallback
 * definitions so the source also builds before SysConfig regenerates its
 * header in a clean CCS workspace.
 */
#if defined(GPIO_SOUND_LIGHT_PORT) && defined(GPIO_SOUND_LIGHT_ALERT_PIN)
#define SOUND_LIGHT_PORT  GPIO_SOUND_LIGHT_PORT
#define SOUND_LIGHT_PIN   GPIO_SOUND_LIGHT_ALERT_PIN
#define SOUND_LIGHT_IOMUX GPIO_SOUND_LIGHT_ALERT_IOMUX
#else
#define SOUND_LIGHT_PORT  GPIOA
#define SOUND_LIGHT_PIN   DL_GPIO_PIN_1
#define SOUND_LIGHT_IOMUX IOMUX_PINCM2
#endif

typedef struct SoundLightStep {
    uint16_t durationMs;
    uint8_t on;
} SoundLightStep;

static const SoundLightStep gKeySteps[] = {
    {40U, 1U},
};

static const SoundLightStep gStartSteps[] = {
    {55U, 1U}, {65U, 0U}, {55U, 1U},
};

static const SoundLightStep gSuccessSteps[] = {
    {80U, 1U}, {80U, 0U}, {180U, 1U},
};

static const SoundLightStep gErrorSteps[] = {
    {180U, 1U}, {100U, 0U}, {180U, 1U},
    {100U, 0U}, {180U, 1U},
};

static const SoundLightStep gPowerOnSteps[] = {
    {60U, 1U}, {70U, 0U}, {60U, 1U},
};

static const SoundLightStep *volatile gSteps;
static volatile uint8_t gStepCount;
static volatile uint8_t gStepIndex;
static volatile uint16_t gRemainingMs;
static volatile bool gActive;

static void SoundLight_setOutput(bool on)
{
    if (on) {
        DL_GPIO_setPins(SOUND_LIGHT_PORT, SOUND_LIGHT_PIN);
    } else {
        DL_GPIO_clearPins(SOUND_LIGHT_PORT, SOUND_LIGHT_PIN);
    }
}

static void SoundLight_start(
    const SoundLightStep *steps, uint8_t stepCount)
{
    gSteps = steps;
    gStepCount = stepCount;
    gStepIndex = 0U;
    gRemainingMs = steps[0].durationMs;
    gActive = true;
    SoundLight_setOutput(steps[0].on != 0U);
}

void SoundLight_init(void)
{
    DL_GPIO_initDigitalOutput(SOUND_LIGHT_IOMUX);
    DL_GPIO_clearPins(SOUND_LIGHT_PORT, SOUND_LIGHT_PIN);
    DL_GPIO_enableOutput(SOUND_LIGHT_PORT, SOUND_LIGHT_PIN);
    gSteps = NULL;
    gStepCount = 0U;
    gStepIndex = 0U;
    gRemainingMs = 0U;
    gActive = false;
}

void SoundLight_play(SoundLightCue cue)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    switch (cue) {
        case SOUND_LIGHT_CUE_KEY:
            SoundLight_start(gKeySteps,
                (uint8_t)(sizeof(gKeySteps) / sizeof(gKeySteps[0])));
            break;
        case SOUND_LIGHT_CUE_START:
            SoundLight_start(gStartSteps,
                (uint8_t)(sizeof(gStartSteps) / sizeof(gStartSteps[0])));
            break;
        case SOUND_LIGHT_CUE_SUCCESS:
            SoundLight_start(gSuccessSteps,
                (uint8_t)(sizeof(gSuccessSteps) / sizeof(gSuccessSteps[0])));
            break;
        case SOUND_LIGHT_CUE_ERROR:
            SoundLight_start(gErrorSteps,
                (uint8_t)(sizeof(gErrorSteps) / sizeof(gErrorSteps[0])));
            break;
        case SOUND_LIGHT_CUE_POWER_ON:
            SoundLight_start(gPowerOnSteps,
                (uint8_t)(sizeof(gPowerOnSteps) / sizeof(gPowerOnSteps[0])));
            break;
        case SOUND_LIGHT_CUE_OFF:
        default:
            gActive = false;
            gRemainingMs = 0U;
            SoundLight_setOutput(false);
            break;
    }
    if (interruptState == 0U) {
        __enable_irq();
    }
}

void SoundLight_forceOff(void)
{
    SoundLight_play(SOUND_LIGHT_CUE_OFF);
}

bool SoundLight_isActive(void)
{
    return gActive;
}

void SoundLight_tick(void)
{
    const SoundLightStep *steps;
    uint8_t nextIndex;

    if (!gActive) {
        return;
    }
    if (gRemainingMs > 0U) {
        gRemainingMs--;
    }
    if (gRemainingMs > 0U) {
        return;
    }

    steps = gSteps;
    nextIndex = (uint8_t)(gStepIndex + 1U);
    if ((steps == NULL) || (nextIndex >= gStepCount)) {
        gActive = false;
        SoundLight_setOutput(false);
        return;
    }

    gStepIndex = nextIndex;
    gRemainingMs = steps[nextIndex].durationMs;
    SoundLight_setOutput(steps[nextIndex].on != 0U);
}
