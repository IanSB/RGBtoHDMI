#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "hw.h"
#include "dma.h"
#include "hdmiaudio.h"
#include "../startup.h"
#include "../logging.h"
#include "../cache.h"
#include "../filesystem.h"
#include "../rgb_to_hdmi.h"

#define TABLE_MAX  100
static const uint16_t wav_table16[] = {
#include "480hz.h"
};

static uint32_t *sound_buffer = (uint32_t *) (UNCACHED_MEM_BASE + 0x1000000);

void start_hdmi_sound(uint32_t *buffer, uint32_t size) {
    log_info("Starting audio, size = %d", size);
    //hdmi_print_regs();
    //hd_print_regs();
    hdmi_audio_setup();
    hdmi_print_regs();
    hd_print_regs();
    start_sound_dma(buffer, size);
    dma_debug(0);
    log_info("Audio setup complete");
}

void stop_audio() {
    //todo investigate proper dma shutdown
    stop_dma(0);
    dma_init();
    hdmi_audio_reset();
}

void start_tone() {
    stop_dma(0);
    dma_init();
    hdmi_audio_reset();
    int sound_buffer_size = 0;
    log_info("Generating Test Tone");
    for (int count = 0; count < 64; count++) {
        for (int sample = 0; sample < TABLE_MAX; sample++) {
            uint32_t data = wav_table16[sample];
            sound_buffer[sound_buffer_size++] = ConvertIEC958Sample16(data);
            sound_buffer[sound_buffer_size++] = ConvertIEC958Sample16(data);
        }
    }
    start_hdmi_sound(sound_buffer, sound_buffer_size << 2);
}

void start_audio(char* filename) {
    stop_dma(0);
    dma_init();
    hdmi_audio_reset();
    unsigned int sound_buffer_size = 0;
    char path[MAX_STRING_SIZE];
    sprintf(path, "/WAVs/%s.wav", filename);
    sound_buffer_size = file_load_WAV(path, sound_buffer);
    start_hdmi_sound(sound_buffer, sound_buffer_size << 2);
}

void start_all_audio(char wav_names[MAX_NAMES][MAX_NAMES_WIDTH], int count) {
    stop_dma(0);
    dma_init();
    hdmi_audio_reset();
    unsigned int sound_buffer_size = 0;
    char path[MAX_STRING_SIZE];
    for (int i = 0; i < count; i++) {
        sprintf(path, "/WAVs/%s.wav", wav_names[i]);
        unsigned int size = file_load_WAV(path, sound_buffer + sound_buffer_size);
        sound_buffer_size += size;
        if (size != 0) {
            for (int j = 0; j < 24000; j++) {         //add 0.5sec silence between wavs
                sound_buffer[sound_buffer_size++] = ConvertIEC958Sample16(0);
                sound_buffer[sound_buffer_size++] = ConvertIEC958Sample16(0);
            }
        }
    }
    start_hdmi_sound(sound_buffer, sound_buffer_size << 2);
}
