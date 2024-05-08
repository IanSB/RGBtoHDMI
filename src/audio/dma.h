#pragma once

#include <stdint.h>

#define DMA_CB_MAX 2560

enum {
	DMA_PERMAP_DEFAULT = 0,
	DMA_PERMAP_DSI_0,
	DMA_PERMAP_PCM_TX,
	DMA_PERMAP_PCM_RX,
	DMA_PERMAP_SMI,
	DMA_PERMAP_PWM,
	DMA_PERMAP_SPI_TX,
	DMA_PERMAP_SPI_RX,
	DMA_PERMAP_BSC_SPI_SLAVE_TX,
	DMA_PERMAP_BSC_SPI_SLAVE_RX,
	DMA_PERMAP_UNUSED,
	DMA_PERMAP_EMMC,
	DMA_PERMAP_UART_TX,
	DMA_PERMAP_SD_HOST,
	DMA_PERMAP_UART_RX,
	DMA_PERMAP_DSI_1,
	DMA_PERMAP_SLIMBUS_MCTX,
	DMA_PERMAP_HDMI,
	DMA_PERMAP_SLIMBUS_MCRX,
	DMA_PERMAP_SLIMBUS_DC0,
	DMA_PERMAP_SLIMBUS_DC1,
	DMA_PERMAP_SLIMBUS_DC2,
	DMA_PERMAP_SLIMBUS_DC3,
	DMA_PERMAP_SLIMBUS_DC4,
	DMA_PERMAP_SCALER_FIFO_0_SMI,
	DMA_PERMAP_SCALER_FIFO_1_SMI,
	DMA_PERMAP_SCALER_FIFO_2_SMI,
	DMA_PERMAP_SLIMBUS_DC5,
	DMA_PERMAP_SLIMBUS_DC6,
	DMA_PERMAP_SLIMBUS_DC7,
	DMA_PERMAP_SLIMBUS_DC8,
	DMA_PERMAP_SLIMBUS_DC9,
};

typedef struct dma_control_block_t {
	uint32_t ti;
	uint32_t s_addr;
	uint32_t d_addr;
	uint32_t txfr_len;
	uint32_t stride;
	struct dma_control_block_t *next_cb;
	uint32_t debug;
	uint32_t reserved;
} __attribute__((__packed__)) dma_control_block;

dma_control_block *dma_get_cb();
void dma_cb_dup(dma_control_block *dst, dma_control_block *src);
void dma_cb_set_addr(dma_control_block *p, uint32_t dst, uint32_t src);
void dma_cb_set_stride_d(dma_control_block *p, uint16_t value);
void dma_cb_set_stride_s(dma_control_block *p, uint16_t value);

void dma_cb_set_ti_burst_length(dma_control_block *p, uint32_t value);
void dma_cb_set_ti_dst_dreq(dma_control_block *p, int value);
void dma_cb_set_ti_dst_inc(dma_control_block *p, int value);
void dma_cb_set_ti_permap(dma_control_block *p, uint32_t value);
void dma_cb_set_ti_src_dreq(dma_control_block *p, int value);
void dma_cb_set_ti_src_inc(dma_control_block *p, int value);
void dma_cb_set_ti_tdmode(dma_control_block *p, int value);
void dma_cb_set_ti_permap(dma_control_block *p, uint32_t value);
void dma_cb_set_txfr_len(dma_control_block *p, uint32_t value);
void dma_cb_set_txfr_len_xlength(dma_control_block *p, uint16_t value);
void dma_cb_set_txfr_len_ylength(dma_control_block *p, uint16_t value);
void dma_clear_cb(dma_control_block *ret);
void dma_debug(int x);
void dma_init();
void dma_submit_cb(int ch);
void dma_wait(int ch);
void start_sound_dma(uint32_t *buffer, uint32_t size);
void stop_dma(int ch);
