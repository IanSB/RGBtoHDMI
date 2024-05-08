#ifndef _HDMIAUDIO_H_
#define _HDMIAUDIO_H_

void hd_print_regs();
void hdmi_print_regs();
void hdmi_audio_prepare();
void hdmi_audio_reset();
void hdmi_audio_setup();
void hdmi_audio_start_packet(int isforce);
void hdmi_audio_startup();
void hdmi_audio_stop_packet(int isforce);
uint32_t hdmi_audio_get_fifo_pointer();
uint32_t ConvertIEC958Sample16(uint32_t nSample);
#endif //_HDMIAUDIO_H_
