#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "max98357.h"
#include "sound_bip.h"
#include "sound_button.h"

typedef struct {
    bool active;
    int16_t *data;
    long length;
    long position;
} TRACK;

typedef struct {
    i2s_chan_handle_t tx_channel;
    int16_t buffer[BUFFER_SIZE_IN_SAMPLE];
    TRACK tracks[TRACK_COUNT];
}s_i2s_ctx;


/* Example configurations */
#define EXAMPLE_SAMPLE_RATE (44100) // 16KHZ
#define EXAMPLE_MCLK_MULTIPLE (384) // If not using 24-bit data width, 256 should be enough
#define EXAMPLE_MCLK_FREQ_HZ (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)
#define EXAMPLE_VOICE_VOLUME CONFIG_EXAMPLE_VOICE_VOLUME
 
/* I2S port and GPIOs */
#define I2S_BCLK GPIO_NUM_15
#define I2S_WS GPIO_NUM_16
#define I2S_D_OUT GPIO_NUM_7
 
//static const char *TAG = "i2s_max98357";

#define CLAMP(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))


volatile unsigned char play_sound_bip_requested = 0;
volatile unsigned char play_sound_button_requested = 0;


s_i2s_ctx  ctx;

void fill_buffer();

void fill_buffer() {
    if (play_sound_bip_requested) {
        play_sound_bip_requested = 0;
        TRACK *t = &ctx.tracks[0];
        t->active = true;
        t->data = (int16_t *)sound_bip_data;
        t->length = SOUND_BIP_SIZE >> 1;
        t->position = 0;
    }
    if (play_sound_button_requested) {
        play_sound_button_requested = 0;
        TRACK *t = &ctx.tracks[1];
        t->active = true;
        t->data = (int16_t *)sound_button_data;
        t->length = SOUND_BUTTON_SIZE >> 1;
        t->position = 0;
    }

    for (int i = 0; i < BUFFER_SIZE_IN_SAMPLE; i++) {
        int sample = 0;
        for (int j = 0; j < TRACK_COUNT; j++) {
            TRACK *t = &ctx.tracks[j];
            if (t->active) {
                sample += t->data[t->position++];
                if (t->position >= t->length) {
                    t->active = false;
                }
            }
        }
        ctx.buffer[i] = CLAMP(sample) / 2;
    }
}

void i2s_create() {

    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_D_OUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_new_channel(&channel_config, &ctx.tx_channel, NULL));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(ctx.tx_channel, &standard_config));
    ESP_ERROR_CHECK(i2s_channel_enable(ctx.tx_channel));
}

void i2s_delete() {

}

void i2s_update() {
    fill_buffer();
    i2s_channel_write(ctx.tx_channel, ctx.buffer, BUFFER_SIZE_IN_SAMPLE << 1, NULL, 1000);
}

void i2s_play_sound_bip(void) {
    play_sound_bip_requested = 1;
}

void i2s_play_sound_button(void) {
    play_sound_button_requested = 1;
}