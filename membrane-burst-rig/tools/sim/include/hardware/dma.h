#pragma once
#include <stdint.h>
#define DMA_SIZE_16 1
#define DREQ_ADC 36
#define DMA_CH0_CTRL_TRIG_EN_BITS 1u
struct sim_dma_ch_t { volatile uint32_t al1_ctrl; };
// INTS registers are write-1-to-clear on the RP2040.
struct sim_w1c_t {
  uint32_t v = 0;
  sim_w1c_t &operator=(uint32_t m) { v &= ~m; return *this; }
  operator uint32_t() const { return v; }
};
struct sim_dma_hw_t { sim_w1c_t ints1; sim_dma_ch_t ch[12]; };
extern sim_dma_hw_t *dma_hw;
struct dma_channel_config { int chain_to; };
dma_channel_config dma_channel_get_default_config(int ch);
void channel_config_set_transfer_data_size(dma_channel_config *c, int);
void channel_config_set_read_increment(dma_channel_config *c, bool);
void channel_config_set_write_increment(dma_channel_config *c, bool);
void channel_config_set_dreq(dma_channel_config *c, int);
void channel_config_set_chain_to(dma_channel_config *c, int ch);
void dma_channel_configure(int ch, const dma_channel_config *c, volatile void *dst, const volatile void *src,
                           uint32_t count, bool start);
void dma_channel_start(int ch);
void dma_channel_abort(int ch);
void dma_channel_set_write_addr(int ch, volatile void *dst, bool trigger);
int dma_claim_unused_channel(bool required);
void dma_channel_set_irq1_enabled(int ch, bool en);
static inline void hw_clear_bits(volatile uint32_t *p, uint32_t m) { *p &= ~m; }
