void stop_all_audio();
void start_tone();
void set_audio_capture(int state);
void start_audio(char *filename);
void initialise_audio_capture_on_VC1();
void start_all_audio(char wav_names[MAX_NAMES][MAX_NAMES_WIDTH], int count);
void update_audio_ctrl();
int get_audio_hardware_type();
