// Based on:
// Raspberry Pi Zero W HDMI Audio 03
// 2022 yasai kumaashi
// https://github.com/kumaashi/RaspberryPI/
// with fixes for GPU L2 cache disabled
/*
 RPA DMA apis.
 DREQ Peripheral
 0 DREQ = 1 This is always on so use this channel if no DREQ is required.
 1 DSI
 2 PCM TX
 3 PCM RX
 4 SMI
 5 PWM
 6 SPI TX
 7 SPI RX
 8 BSC/SPI Slave TX
 9 BSC/SPI Slave RX
 10 unused
 11 e.MMC
 12 UART TX
 13 SD HOST
 14 UART RX.
 15 DSI
 16 SLIMBUS MCTX.
 17 HDMI
 18 SLIMBUS MCRX
 19 SLIMBUS DC0
 20 SLIMBUS DC1
 21 SLIMBUS DC2
 22 SLIMBUS DC3
 23 SLIMBUS DC4
 24 Scaler FIFO 0 & SMI *
 25 Scaler FIFO 1 & SMI *
 26 Scaler FIFO 2 & SMI *
 27 SLIMBUS DC5
 28 SLIMBUS DC6
 29 SLIMBUS DC7
 30 SLIMBUS DC8
 31 SLIMBUS DC9
*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "hw.h"
#include "dma.h"
#include "../startup.h"
#include "../logging.h"
#include "../cache.h"

#define DMA_CS(x)                  ((volatile uint32_t *)(_get_peripheral_base() + 0x7000 + (0x100 * (x))))
#define DMA_CB_ADDR(x)             ((volatile uint32_t *)(_get_peripheral_base() + 0x7004 + (0x100 * (x))))
#define DMA_TI(x)                  ((volatile uint32_t *)(_get_peripheral_base() + 0x7008 + (0x100 * (x))))
#define DMA_S_ADDR(x)              ((volatile uint32_t *)(_get_peripheral_base() + 0x700c + (0x100 * (x))))
#define DMA_D_ADDR(x)              ((volatile uint32_t *)(_get_peripheral_base() + 0x7010 + (0x100 * (x))))
#define DMA_TXFR_LEN(x)            ((volatile uint32_t *)(_get_peripheral_base() + 0x7014 + (0x100 * (x))))
#define DMA_STRIDE(x)              ((volatile uint32_t *)(_get_peripheral_base() + 0x7018 + (0x100 * (x))))
#define DMA_NEXTCONBK(x)           ((volatile uint32_t *)(_get_peripheral_base() + 0x701c + (0x100 * (x))))
#define DMA_DEBUG(x)               ((volatile uint32_t *)(_get_peripheral_base() + 0x7020 + (0x100 * (x))))
#define DMA_INT_STATUS             ((volatile uint32_t *)(_get_peripheral_base() + 0x7FE0))
#define DMA_ENABLE                 ((volatile uint32_t *)(_get_peripheral_base() + 0x7FF0))

#define GPU_MEMORY_OFFSET 0xc0000000         //required when GPU L2 cache disabled to avoid clicks in audio. Maybe change to 0x4 or 0x8 if GPU L2 cache enabled
#define GPU_MEMORY_OFFSET_MASK 0x3fffffff

//static dma_control_block g_dma_cbs[DMA_CB_MAX]  __attribute__ ((aligned (32))) ;
static dma_control_block *g_dma_cbs = (dma_control_block*) (UNCACHED_MEM_BASE + 0xf0000);  //todo: investigate some problems with cached control block
struct dma_control_block_t *g_dma_cb_head;
static int dma_cb_index;

void dma_clear_cb(dma_control_block *ret) {
	ret->ti = 0;
	ret->s_addr = 0;
	ret->d_addr = 0;
	ret->txfr_len = 0;
	ret->stride = 0;
	ret->next_cb = 0;
	ret->debug = 0;
	ret->reserved = 0;
}

void dma_cb_dup(dma_control_block *dst, dma_control_block *src)
{
	dst->ti = src->ti;
	dst->s_addr = src->s_addr;
	dst->d_addr =src->d_addr;
	dst->txfr_len = src->txfr_len;
	dst->stride = src->stride;
	dst->next_cb = src->next_cb;
	dst->debug = src->debug;
	dst->reserved =src->reserved;
}
void dma_init() {
	dma_cb_index = 0;
	g_dma_cb_head = g_dma_cbs;
	for(int i = 0 ; i < DMA_CB_MAX; i++) {
		dma_clear_cb(&g_dma_cbs[i]);
	}
}

void dma_cb_set_addr(dma_control_block *p, uint32_t dst, uint32_t src) {
	p->d_addr = (uint32_t)dst;
	p->s_addr = (uint32_t)src;

	if(p->s_addr < VCADDR_BASE)
		p->s_addr += VCADDR_BASE;
	if(p->d_addr < VCADDR_BASE)
		p->d_addr += VCADDR_BASE;

}

void dma_cb_set_ti_permap(dma_control_block *p, uint32_t value) {
	p->ti |= value << 16;
}

void dma_cb_set_ti_burst_length(dma_control_block *p, uint32_t value) {
	p->ti |= (value & 0xF) << 12;
}

void dma_cb_set_ti_src_dreq(dma_control_block *p, int value) {
	p->ti |= ((value ? 1 : 0) << 10);
}

void dma_cb_set_ti_src_inc(dma_control_block *p, int value) {
	p->ti |= ((value ? 1 : 0) << 8);
}

void dma_cb_set_ti_dst_dreq(dma_control_block *p, int value) {
	p->ti |= ((value ? 1 : 0) << 6);
}

void dma_cb_set_ti_dst_inc(dma_control_block *p, int value) {
	p->ti |= ((value ? 1 : 0) << 4);
}

void dma_cb_set_ti_tdmode(dma_control_block *p, int value) {
	p->ti |= ((value ? 1 : 0) << 1);
}

void dma_cb_set_txfr_len(dma_control_block *p, uint32_t value) {
	p->txfr_len = value;
}

void dma_cb_set_txfr_len_xlength(dma_control_block *p, uint16_t value) {
	p->txfr_len &= 0xFFFF0000;
	p->txfr_len |= value;
}

void dma_cb_set_txfr_len_ylength(dma_control_block *p, uint16_t value) {
	p->txfr_len &= 0x0000FFFF;
	p->txfr_len |= value << 16;
}

void dma_cb_set_stride_d(dma_control_block *p, uint16_t value) {
	p->stride &= 0x0000FFFF;
	p->stride |= value << 16;
}

void dma_cb_set_stride_s(dma_control_block *p, uint16_t value) {
	p->stride &= 0xFFFF0000;
	p->stride |= value;
}

dma_control_block *dma_get_cb() {
	dma_control_block * ret = &g_dma_cbs[dma_cb_index % DMA_CB_MAX];
	dma_clear_cb(ret);
	dma_cb_index++;

	ret->next_cb = &g_dma_cbs[dma_cb_index % DMA_CB_MAX];
	dma_clear_cb(ret->next_cb);
	return ret;
}

void dma_debug(int x) {
	log_info("----------------------------------------------------------------");
    log_info("DMA : DMA_INT_STATUS   (%08X) : %08X", DMA_INT_STATUS, *DMA_INT_STATUS);
    log_info("DMA : DMA_ENABLE       (%08X) : %08X", DMA_ENABLE, *DMA_ENABLE);
    log_info("DMA : DMA_CS(%d)        (%08X) : %08X", x, DMA_CS(x), *DMA_CS(x));
    log_info("DMA : DMA_CB_ADDR(%d)   (%08X) : %08X", x, DMA_CB_ADDR(x), *DMA_CB_ADDR(x));
    log_info("DMA : DMA_TI(%d)        (%08X) : %08X", x, DMA_TI(x), *DMA_TI(x));
    log_info("DMA : DMA_S_ADDR(%d)    (%08X) : %08X", x, DMA_S_ADDR(x), *DMA_S_ADDR(x));
    log_info("DMA : DMA_D_ADDR(%d)    (%08X) : %08X", x, DMA_D_ADDR(x), *DMA_D_ADDR(x));
    log_info("DMA : DMA_TXFR_LEN(%d)  (%08X) : %08X", x, DMA_TXFR_LEN(x), *DMA_TXFR_LEN(x));
    log_info("DMA : DMA_STRIDE(%d)    (%08X) : %08X", x, DMA_STRIDE(x), *DMA_STRIDE(x));
    log_info("DMA : DMA_NEXTCONBK(%d) (%08X) : %08X", x, DMA_NEXTCONBK(x), *DMA_NEXTCONBK(x));
    log_info("DMA : DMA_DEBUG(%d)     (%08X) : %08X", x, DMA_DEBUG(x), *DMA_DEBUG(x));
    log_info("DMA : g_dma_cb_head               : %08X",(uint32_t)g_dma_cb_head);
	log_info("----------------------------------------------------------------");
}

void dma_submit_cb(int ch) {
	*DMA_CS(ch) = (1 << 31);
	*DMA_CB_ADDR(ch) = (uint32_t)g_dma_cb_head | GPU_MEMORY_OFFSET;    //the control block pointers have to be at GPU_MEMORY_OFFSET in GPU address space when audio data is also at that offset
//	/InvalidateData();
    //CleanDataCache();
	*DMA_CS(ch) |= (1 << 28);
	*DMA_CS(ch) |= (1 << 0);
	dma_cb_index++;
	g_dma_cb_head = &g_dma_cbs[dma_cb_index % DMA_CB_MAX];

}

void stop_dma(int ch) {
	*DMA_CS(ch) = (1 << 31);
    dma_init();
}

void dma_wait(int ch) {
	int count = 0;
	while((*DMA_CS(ch)) & 0x01) {
		count++;
		if(count > 0x100000) {
			log_info("DMA : hang DMA_CS=", *DMA_CS(ch));
			log_info("DMA : hang ch=", ch);
			break;
		}
	}
}

void start_sound_dma(uint32_t *buffer, uint32_t size) {
    dma_init();
    dma_control_block *dmadata = dma_get_cb();
    dma_cb_set_addr(dmadata, (uint32_t)HDMI_MAI_DATA_BUS, (uint32_t) buffer | GPU_MEMORY_OFFSET); //the audio data has to be at GPU_MEMORY_OFFSET in GPU address space to avoid clicks when L2 cache disabled
    dma_cb_set_ti_src_inc(dmadata, 1);
    dma_cb_set_ti_burst_length(dmadata, 2);
    dma_cb_set_txfr_len(dmadata, size);
    //enable dreq from HDMI
    dma_cb_set_ti_dst_dreq(dmadata, 1);
    dma_cb_set_ti_permap(dmadata, DMA_PERMAP_HDMI);
    //make ring link
    dma_control_block *dmadata2 = dma_get_cb();
    dma_cb_dup(dmadata2, dmadata);
    dmadata->next_cb = (dma_control_block*) ((uint32_t) dmadata->next_cb | GPU_MEMORY_OFFSET);   //the control block pointers have to be at GPU_MEMORY_OFFSET in GPU address space when audio data is also at that offset
    dmadata2->next_cb = (dma_control_block*) ((uint32_t) dmadata | GPU_MEMORY_OFFSET);           //the control block pointers have to be at GPU_MEMORY_OFFSET in GPU address space when audio data is also at that offset
    //kick
    dma_submit_cb(0);
}
