#ifndef RGB_TO_HDMI_H
#define RGB_TO_HDMI_H

// Property setters/getters
void set_config_overscan(int l, int r, int t, int b);
void get_config_overscan(int *l, int *r, int *t, int *b);
void set_startup_overscan(int value);
int get_startup_overscan();
void set_force_genlock_range(int value);
void set_auto_workaround_path(char *value, int reboot);
void reboot(void);

void set_resolution(int mode, const char *name, int reboot);
void set_refresh(int value, int reboot);
void set_hdmi(int value, int reboot);
void set_hdmi_auto(int value, int reboot);
void set_scaling(int mode, int reboot);
void set_frontend(int value, int save);
void set_general_reboot();
void set_ntsccolour(int value);
void set_timingset(int value);
int  get_adjusted_ntscphase();
int  get_lines_per_vsync(int compensate);
int  get_one_line_time_ns();
int  get_vsync_width_lines();
int  get_sync_detected();
int  get_50hz_state();
int  get_core_1_available();

void set_parameter(int parameter, int value);
int get_parameter(int parameter);

int show_detected_status(int line);
void delay_in_arm_cycles_cpu_adjust(int cycles);
void set_filtering(int filter);
int get_current_display_buffer();
void set_vsync_psync(int state);
// Actions
void action_calibrate_clocks();
void action_calibrate_auto();
void calculate_cpu_timings();
int read_cpld_version();
// Status
int is_genlocked();
void set_status_message(char *msg);
void force_reinit();
void set_helper_flag();
int eight_bit_detected();
int new_DAC_detected();
int any_DAC_detected();
int mono_board_detected();
int extra_flags();
int calibrate_sampling_clock(int profile_changed);
void DPMS(int dpms_state);
void start_vc_bench(int type);
void refresh_cpld();
// Reboot the system immediately
void reboot();

#endif
