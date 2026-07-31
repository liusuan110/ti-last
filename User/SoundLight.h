#ifndef SOUND_LIGHT_H
#define SOUND_LIGHT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum SoundLightCue {
    SOUND_LIGHT_CUE_OFF = 0,
    SOUND_LIGHT_CUE_KEY,
    SOUND_LIGHT_CUE_START,
    SOUND_LIGHT_CUE_SUCCESS,
    SOUND_LIGHT_CUE_ERROR,
    SOUND_LIGHT_CUE_POWER_ON
} SoundLightCue;

void SoundLight_init(void);
void SoundLight_play(SoundLightCue cue);
void SoundLight_forceOff(void);
bool SoundLight_isActive(void);
void SoundLight_tick(void);

#endif
