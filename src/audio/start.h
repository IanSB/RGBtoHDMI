void stop_all_audio();
void start_tone();
void set_audio_capture(int state);
void start_audio(char *filename);
void initialise_audio_capture_on_VC1();
void start_all_audio(char wav_names[MAX_NAMES][MAX_NAMES_WIDTH], int count);
void update_audio_ctrl();
int get_audio_hardware_type();
enum {
    AUDIO_2GPIO_24KHZ_L,
    AUDIO_2GPIO_24KHZ_F,
    AUDIO_2GPIO_48KHZ_L,
    AUDIO_2GPIO_48KHZ_F,
    AUDIO_1GPIO_24KHZ_L,
    AUDIO_1GPIO_24KHZ_F,
    AUDIO_1GPIO_48KHZ_L,
    AUDIO_1GPIO_48KHZ_F
};
#define AUDIO_NO_HARDWARE 0xffffffff
#define AUDIO_PINS 1
#define AUDIO_48KHZ 2
#define AUDIO_1GPIO 4