#ifndef SPEAKER_H
#define SPEAKER_H

#include <stdint.h>

typedef enum { WAVE_SINE = 0, WAVE_TRIANGLE = 1, WAVE_SQUARE = 2, WAVE_SAWTOOTH = 3 } speaker_waveform_t;

void speaker_init(void);
void speaker_set_volume(uint8_t volume);
void speaker_set_waveform(speaker_waveform_t wave);
void speaker_play_frequency(uint16_t freq);
void speaker_mute(void);
void speaker_stop(void);
uint8_t speaker_is_playing(void);

#endif /* SPEAKER_H */