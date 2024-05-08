// Based on:
// Raspberry Pi Zero W HDMI Audio 03
// 2022 yasai kumaashi
// https://github.com/kumaashi/RaspberryPI/
// and circle bare metal OS
// https://github.com/rsta2/circle
// with additions and bug fixes

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "hw.h"
#include "../logging.h"
#include "../startup.h"
#include "../defs.h"
#include "../info.h"

void hd_print_regs() {
    log_info("-------------------------------------------");
    log_info("HDMI_M_CTL         = %08X", *HDMI_M_CTL       );
    log_info("HDMI_MAI_CTL       = %08X", *HDMI_MAI_CTL     );
    log_info("HDMI_MAI_THR       = %08X", *HDMI_MAI_THR     );
    log_info("HDMI_MAI_FMT       = %08X", *HDMI_MAI_FMT     );
    log_info("HDMI_MAI_DATA      = %08X", *HDMI_MAI_DATA    );
    log_info("HDMI_MAI_SMP       = %08X", *HDMI_MAI_SMP     );
    log_info("HDMI_VID_CTL       = %08X", *HDMI_VID_CTL     );
    log_info("HDMI_CSC_CTL       = %08X", *HDMI_CSC_CTL     );
    log_info("HDMI_CSC_12_11     = %08X", *HDMI_CSC_12_11   );
    log_info("HDMI_CSC_14_13     = %08X", *HDMI_CSC_14_13   );
    log_info("HDMI_CSC_22_21     = %08X", *HDMI_CSC_22_21   );
    log_info("HDMI_CSC_24_23     = %08X", *HDMI_CSC_24_23   );
    log_info("HDMI_CSC_32_31     = %08X", *HDMI_CSC_32_31   );
    log_info("HDMI_CSC_34_33     = %08X", *HDMI_CSC_34_33   );
    log_info("HDMI_FRAME_COUNT   = %08X", *HDMI_FRAME_COUNT );
    log_info("-------------------------------------------");
}

void hdmi_print_regs() {
    log_info("-------------------------------------------");
    log_info("HDMI_CORE_REV             = %08X",              *HDMI_CORE_REV            );
    log_info("HDMI_SW_RESET_CONTROL     = %08X",              *HDMI_SW_RESET_CONTROL    );
    log_info("HDMI_HOTPLUG_INT          = %08X",              *HDMI_HOTPLUG_INT         );
    log_info("HDMI_HOTPLUG              = %08X",              *HDMI_HOTPLUG             );
    log_info("HDMI_FIFO_CTL             = %08X",              *HDMI_FIFO_CTL            );
    log_info("HDMI_MAI_CHANNEL_MAP      = %08X",              *HDMI_MAI_CHANNEL_MAP     );
    log_info("HDMI_MAI_CONFIG           = %08X",              *HDMI_MAI_CONFIG          );
    log_info("HDMI_MAI_FORMAT           = %08X",              *HDMI_MAI_FORMAT          );
    log_info("HDMI_AUDIO_PACKET_CONFIG  = %08X",              *HDMI_AUDIO_PACKET_CONFIG );
    log_info("HDMI_RAM_PACKET_CONFIG    = %08X",              *HDMI_RAM_PACKET_CONFIG   );
    log_info("HDMI_RAM_PACKET_STATUS    = %08X",              *HDMI_RAM_PACKET_STATUS   );
    log_info("HDMI_CRP_CFG              = %08X",              *HDMI_CRP_CFG             );
    log_info("HDMI_CTS_0                = %08X",              *HDMI_CTS_0               );
    log_info("HDMI_CTS_1                = %08X",              *HDMI_CTS_1               );
    log_info("HDMI_SCHEDULER_CONTROL    = %08X",              *HDMI_SCHEDULER_CONTROL   );
    log_info("HDMI_HORZA                = %08X",              *HDMI_HORZA               );
    log_info("HDMI_HORZB                = %08X",              *HDMI_HORZB               );
    log_info("HDMI_VERTA0               = %08X",              *HDMI_VERTA0              );
    log_info("HDMI_VERTB0               = %08X",              *HDMI_VERTB0              );
    log_info("HDMI_VERTA1               = %08X",              *HDMI_VERTA1              );
    log_info("HDMI_VERTB1               = %08X",              *HDMI_VERTB1              );
    log_info("HDMI_CEC_CNTRL_1          = %08X",              *HDMI_CEC_CNTRL_1         );
    log_info("HDMI_CEC_CNTRL_2          = %08X",              *HDMI_CEC_CNTRL_2         );
    log_info("HDMI_CEC_CNTRL_3          = %08X",              *HDMI_CEC_CNTRL_3         );
    log_info("HDMI_CEC_CNTRL_4          = %08X",              *HDMI_CEC_CNTRL_4         );
    log_info("HDMI_CEC_CNTRL_5          = %08X",              *HDMI_CEC_CNTRL_5         );
    log_info("HDMI_CEC_TX_DATA_1        = %08X",              *HDMI_CEC_TX_DATA_1       );
    log_info("HDMI_CEC_TX_DATA_2        = %08X",              *HDMI_CEC_TX_DATA_2       );
    log_info("HDMI_CEC_TX_DATA_3        = %08X",              *HDMI_CEC_TX_DATA_3       );
    log_info("HDMI_CEC_TX_DATA_4        = %08X",              *HDMI_CEC_TX_DATA_4       );
    log_info("HDMI_CEC_RX_DATA_1        = %08X",              *HDMI_CEC_RX_DATA_1       );
    log_info("HDMI_CEC_RX_DATA_2        = %08X",              *HDMI_CEC_RX_DATA_2       );
    log_info("HDMI_CEC_RX_DATA_3        = %08X",              *HDMI_CEC_RX_DATA_3       );
    log_info("HDMI_CEC_RX_DATA_4        = %08X",              *HDMI_CEC_RX_DATA_4       );
    log_info("HDMI_TX_PHY_RESET_CTL     = %08X",              *HDMI_TX_PHY_RESET_CTL    );
    log_info("HDMI_TX_PHY_CTL_0         = %08X",              *HDMI_TX_PHY_CTL_0        );
    log_info("HDMI_CEC_CPU_STATUS       = %08X",              *HDMI_CEC_CPU_STATUS      );
    log_info("HDMI_CEC_CPU_SET          = %08X",              *HDMI_CEC_CPU_SET         );
    log_info("HDMI_CEC_CPU_CLEAR        = %08X",              *HDMI_CEC_CPU_CLEAR       );
    log_info("HDMI_CEC_CPU_MASK_STATUS  = %08X",              *HDMI_CEC_CPU_MASK_STATUS );
    log_info("HDMI_CEC_CPU_MASK_SET     = %08X",              *HDMI_CEC_CPU_MASK_SET    );
    log_info("HDMI_CEC_CPU_MASK_CLEAR   = %08X",              *HDMI_CEC_CPU_MASK_CLEAR  );
    log_info("HDMI_RAM_PACKET_START     = %08X",              *HDMI_RAM_PACKET_START    );
    log_info("-------------------------------------------");
}

// ConvertIEC958 derived from  https://github.com/rsta2/circle
#define IEC958_HW_CHANNELS      2       // more channels unsupported
#define IEC958_FRAMES_PER_BLOCK     192
#define IEC958_SUBFRAMES_PER_BLOCK  (IEC958_HW_CHANNELS * IEC958_FRAMES_PER_BLOCK)
#define IEC958_STATUS_BYTES     5
#define IEC958_B_FRAME_PREAMBLE     0x0f // 0x08 in linux and 0x0f in circle
#define CM_DIV_FRAC_BITS	12
#define BIT(x) (1 << (x))


static volatile uint32_t *gpioreg = 0;

int parity32 (unsigned nValue)
{
    int nResult = 0;

    while (nValue != 0)
    {
        nResult ^= (nValue & 1);
        nValue >>= 1;
    }

    return nResult;
}

uint32_t ConvertIEC958Sample16(uint32_t nSample) {
    static uint32_t m_nSubFrame = 0;
    static uint8_t m_uchIEC958Status[IEC958_STATUS_BYTES] = {
                0b100,   // consumer, PCM, no copyright, no pre-emphasis
                    0,   // category (general mode)
                    0,   // source number, take no account of channel number
                    2,   // sampling frequency uchFS = 2 = 48000
    0b1011 | (13 << 4)   // 24 bit samples, original freq.  uchOrigFS = 13 = 48000
    };
    uint32_t nFrame = m_nSubFrame / IEC958_HW_CHANNELS;

	nSample &= 0xFFFF;
	nSample <<= 12;     //8 for 16 to 24 bit + 4 subframe bits

    if (   nFrame < IEC958_STATUS_BYTES * 8
        && (m_uchIEC958Status[nFrame / 8] & BIT(nFrame % 8))) {
        nSample |= 0x40000000;
    }

    if (parity32(nSample )) {
        nSample  |= 0x80000000;
    }

    if (nFrame == 0) {
        nSample |= IEC958_B_FRAME_PREAMBLE;
    }

    if (++m_nSubFrame == IEC958_SUBFRAMES_PER_BLOCK) {
        m_nSubFrame = 0;
    }

    return nSample ;
}


uint32_t hdmi_audio_get_fifo_pointer() {
    return (uint32_t)HDMI_MAI_DATA_BUS;
}

void hdmi_audio_stop_packet(int isforce) {
    return;
    log_info("hdmi_audio_stop_packet");
    *HDMI_RAM_PACKET_CONFIG &= ~(1 << 4);           //Checked (AudioPacketIdentifier bit)
    if(isforce)
        return;
    while(1) {
        if(*HDMI_RAM_PACKET_STATUS & (1 << 4))      //Checked (AudioPacketIdentifier bit)
            break;
    };
}

void hdmi_audio_start_packet(int isforce) {
    log_info("hdmi_audio_start_packet");
    *HDMI_RAM_PACKET_CONFIG |= (1 << 4);             //Checked (AudioPacketIdentifier bit)
    if(isforce)
        return;
    while(1) {
        if(*HDMI_RAM_PACKET_STATUS & (1 << 4))       //Checked (AudioPacketIdentifier bit)
            break;
    };
}

void hdmi_audio_reset() {
    log_info("hdmi_audio_reset");
    uint32_t mai_ctl = 0;
    hdmi_audio_stop_packet(1);
    mai_ctl = (1 << 0);  //RST
    mai_ctl = (1 << 2);  //UF
    mai_ctl = (1 << 9);  //FLUSH
    *HDMI_MAI_CTL = mai_ctl;        //checked against circle
}

void hdmi_audio_startup() {
    log_info("hdmi_audio_startup");
    uint32_t mai_ctl = 0;
    mai_ctl |= (1 << 0);  //RST
    mai_ctl |= (1 << 1);  //OF
    mai_ctl |= (1 << 2);  //UF
    mai_ctl |= (1 << 15); //DLATE
    mai_ctl |= (1 << 9);  //FLUSH
    *HDMI_MAI_CTL = mai_ctl;         //checked against circle
}

/*
 * From linux/lib/math/rational.c:
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 * rational fractions
 *
 * Copyright (C) 2009 emlix GmbH, Oskar Schirmer <oskar@scara.com>
 * Copyright (C) 2019 Trent Piepho <tpiepho@gmail.com>
 *
 * helper functions when coping with rational numbers
 */

/*
 * calculate best rational approximation for a given fraction
 * taking into account restricted register size, e.g. to find
 * appropriate values for a pll with 5 bit denominator and
 * 8 bit numerator register fields, trying to set up with a
 * frequency ratio of 3.1415, one would say:
 *
 * rational_best_approximation(31415, 10000,
 *		(1 << 8) - 1, (1 << 5) - 1, &n, &d);
 *
 * you may look at given_numerator as a fixed point number,
 * with the fractional part size described in given_denominator.
 *
 * for theoretical background, see:
 * https://en.wikipedia.org/wiki/Continued_fraction
 */

#define min(a, b)	(((a) < (b)) ? (a) : (b))

void rational_best_approximation(
	unsigned long given_numerator, unsigned long given_denominator,
	unsigned long max_numerator, unsigned long max_denominator,
	unsigned long *best_numerator, unsigned long *best_denominator) {
	// n/d is the starting rational, which is continually
	// decreased each iteration using the Euclidean algorithm.

	// dp is the value of d from the prior iteration.

	// n2/d2, n1/d1, and n0/d0 are our successively more accurate
	// approximations of the rational.  They are, respectively,
	// the current, previous, and two prior iterations of it.

	// a is current term of the continued fraction.

	unsigned long n, d, n0, d0, n1, d1, n2, d2;
	n = given_numerator;
	d = given_denominator;
	n0 = d1 = 0;
	n1 = d0 = 1;

	for (;;) {
		unsigned long dp, a;

		if (d == 0)
			break;
		// Find next term in continued fraction, 'a', via
		// Euclidean algorithm.

		dp = d;
		a = n / d;
		d = n % d;
		n = dp;

		// Calculate the current rational approximation (aka
		// convergent), n2/d2, using the term just found and
		// the two prior approximations.

		n2 = n0 + a * n1;
		d2 = d0 + a * d1;

		// If the current convergent exceeds the maxes, then
		// return either the previous convergent or the
		// largest semi-convergent, the final term of which is
		// found below as 't'.

		if ((n2 > max_numerator) || (d2 > max_denominator)) {
			unsigned long t = min((max_numerator - n0) / n1,
					      (max_denominator - d0) / d1);

			// This tests if the semi-convergent is closer
			// than the previous convergent.

			if (2u * t > a || (2u * t == a && d0 * dp > d1 * d)) {
				n1 = n0 + t * n1;
				d1 = d0 + t * d1;
			}
			break;
		}
		n0 = n1;
		n1 = n2;
		d0 = d1;
		d1 = d2;
	}
	*best_numerator = n1;
	*best_denominator = d1;
}


double get_pllh_clock_rate(void) {
double pllh_clock;
#ifdef RPI4
   volatile uint32_t* pi4_hdmi0_regs;
   pi4_hdmi0_regs = PI4_HDMI0_PLL;
   pllh_clock = (double) 1000000UL * (pi4_hdmi0_regs[PI4_HDMI0_RM_OFFSET] & 0x7fffffff);
#else
   gpioreg = (volatile uint32_t *)(_get_peripheral_base() + 0x101000UL);
   int PLLH_ANA1_PREDIV = ((gpioreg[PLLH_ANA1] >> 11) & 1) ? 2 : 1; //prediv on bit 11 instead of bit 14 for pllh
   pllh_clock = (CRYSTAL * ((double)(gpioreg[PLLH_CTRL] & 0x3ff) + ((double)gpioreg[PLLH_FRAC]) / ((double)(1 << 20)))) * PLLH_ANA1_PREDIV;
#endif
   log_info("pllh Clock: %lf MHz", pllh_clock);
   return pllh_clock;
}

double get_pixel_clock_rate(void) {
   double pllh_clock = get_pllh_clock_rate() * 1000000UL;
   int fixed_divider = 10;
#ifdef RPI4
   volatile uint32_t* pi4_hdmi0_regs;
   pi4_hdmi0_regs = PI4_HDMI0_PLL;
   static double divider = 1;
   divider = (double) ((pi4_hdmi0_regs[PI4_HDMI0_DIVIDER] & 0x0000ff00) >> 8);
   // conversion of pllh_clock to pixel clock for pi4 derived from
   // https://github.com/torvalds/linux/blob/master/drivers/gpu/drm/vc4/vc4_hdmi_phy.c#L161
   // https://github.com/torvalds/linux/blob/master/drivers/gpu/drm/vc4/vc4_hdmi_phy.c#L189
   double pixel_clock = pllh_clock * CRYSTAL / 0x200000 / divider / fixed_divider;
#else
   gpioreg = (volatile uint32_t *)(_get_peripheral_base() + 0x101000UL);
   // PLLH seems to use a fixed divider to generate the pixel clock
   // An additional divider is used to get very low pixel clock rates ^
   int additional_divider = gpioreg[PLLH_PIX];
   //log_debug("Additional divider: %d", additional_divider);
   // Calculate the pixel clock
   double pixel_clock = pllh_clock / ((double) fixed_divider) / ((double) additional_divider);
#endif
   log_info("Pixel Clock: %lf Hz", pixel_clock);
   return pixel_clock;
}

unsigned long GetHSMClockRate(void)
{
    gpioreg = (volatile uint32_t *)(_get_peripheral_base() + 0x101000UL);

#ifdef RPI4
	unsigned long ulParentRate = 750000000;   //GPIOClockSourcePLLD
#else
	unsigned long ulParentRate = 500000000;   //GPIOClockSourcePLLD
#endif

	uint32_t div = gpioreg[CM_HSMDIV];

	// The divisor is a 12.12 fixed point field, but only some of
	// the bits are populated in any given clock.

	static const uint32_t int_bits = 4;
	static const uint32_t frac_bits = 8;
	div >>= CM_DIV_FRAC_BITS - frac_bits;
	div &= (1 << (int_bits + frac_bits)) - 1;

	if (div == 0)
	{
		return 0;
	}

	uint64_t rate = (uint64_t) ulParentRate << frac_bits;
	rate /= div;

	return rate;
}

void hdmi_audio_prepare() {
    log_info("hdmi_audio_prepare");

    unsigned long m_ulAudioClockRate = GetHSMClockRate();
    unsigned long m_nSampleRate = 48000;
    unsigned long ulNumerator, ulDenominator;
	rational_best_approximation (m_ulAudioClockRate, m_nSampleRate,
				     0xFFFFFFU,
				     0xFFU + 1,
				     &ulNumerator, &ulDenominator);

    log_info("HSM = %d, n=%d, d=%d", (uint32_t) m_ulAudioClockRate, (uint32_t) ulNumerator, (uint32_t) ulDenominator );
    log_info("val = %lf",  ((double)m_ulAudioClockRate*ulDenominator/ulNumerator));
    uint32_t mai_smp = (uint32_t) ulNumerator << 8
				     | (uint32_t) (ulDenominator - 1);
    log_info("calculated MAI_SMP = %08X", mai_smp);

    *HDMI_MAI_SMP = mai_smp;

    //MAI
    uint32_t mai_ctl = 0;
    int ch = 2; //koko
    mai_ctl |= (1 << 3);
    mai_ctl |= (ch << 4);
    mai_ctl |= (1 << 12);
    mai_ctl |= (1 << 13);
    *HDMI_MAI_CTL = mai_ctl; //checked against circle

    *HDMI_MAI_FMT = 0x20900; //ch:2, fs:48000hz //checked against circle

    *HDMI_MAI_THR = 0x08080608; //DREQ     //checked against linux, was 0x10101010 in circle but causes glitches in video capture
 	//REGSHIFT (MaiThreshold, DREQLow, 0);
	//REGSHIFT (MaiThreshold, DREQHigh, 8);
	//REGSHIFT (MaiThreshold, PanicLow, 16);
	//REGSHIFT (MaiThreshold, PanicHigh, 24);

    *HDMI_MAI_CONFIG = (1 << 27) | (1 << 26) | (1 << 1) | (1 << 0);    //checked against circle

#ifdef RPI4
    *HDMI_MAI_CHANNEL_MAP = 0b00010000;    //from circle
#else
    *HDMI_MAI_CHANNEL_MAP =   0b001000;    //checked against circle
#endif

    uint32_t cfg = 0;
    cfg |= (1 << 29); // ZERO DATA SAMPLE_FLAT
    cfg |= (1 << 24); // ZERO DATA INACTIVE CH
    cfg |= (IEC958_B_FRAME_PREAMBLE << 10); //changed to support IEC958 frames as some TVs require the audio samples to be formatted this way
    cfg |= (1 << 1) | (1 << 0); //Left, Right

    log_info("audio config=%08X", cfg);

    *HDMI_AUDIO_PACKET_CONFIG = cfg;  //checked against circle

	uint32_t nSampleRateMul128 = m_nSampleRate * 128;
    int cts_n = nSampleRateMul128 / 1000; //0x1800
    *HDMI_CRP_CFG = cts_n | (1 << 24); //EXTERNAL CTS EN   //checked against circle

    unsigned long pixel_clock = get_pixel_clock_rate();

	uint32_t nCTS = (uint32_t) (((uint64_t) pixel_clock * cts_n) / nSampleRateMul128);
    log_info("nCTS = %08x",nCTS);

    *HDMI_CTS_0 = nCTS; //0x1220A;
    *HDMI_CTS_1 = nCTS; //0x1220A;

    //Write Frame
    *HDMI_RAM_PACKET_CONFIG |= (1 << 16);              //checked against circle (enable bit)
    hdmi_audio_stop_packet(0);
    *HDMI_RAM_PACKET(0, (9 * 4) + 0) = 0x000A0184;     //checked against circle
    *HDMI_RAM_PACKET(0, (9 * 4) + 1) = 0x00000170;
    *HDMI_RAM_PACKET(0, (9 * 4) + 2) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 4) + 3) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 4) + 4) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 4) + 5) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 4) + 6) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 4) + 7) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 4) + 8) = 0x00000000;

    *HDMI_RAM_PACKET(0, (9 * 5) + 0) = 0x000A0184;
    *HDMI_RAM_PACKET(0, (9 * 5) + 1) = 0x00000170;
    *HDMI_RAM_PACKET(0, (9 * 5) + 2) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 5) + 3) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 5) + 4) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 5) + 5) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 5) + 6) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 5) + 7) = 0x00000000;
    *HDMI_RAM_PACKET(0, (9 * 5) + 8) = 0x00000000;
    hdmi_audio_start_packet(0);
}

void hdmi_audio_setup() {
    hdmi_audio_reset();
    hdmi_audio_startup();
    hdmi_audio_prepare();
}
