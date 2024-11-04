#ifndef __MAX98357_H__
#define __MAX98357_H__

#include <inttypes.h>

#define BUFFER_SIZE_IN_SAMPLE 128
#define TRACK_COUNT 2

void i2s_create();
void i2s_update();
void i2s_play_sound_bip(void);
void i2s_play_sound_button(void);


#endif
