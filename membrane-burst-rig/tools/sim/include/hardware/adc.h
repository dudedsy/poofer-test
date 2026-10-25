#pragma once
#include <stdint.h>
struct sim_adc_hw_t { volatile uint32_t fifo; };
extern sim_adc_hw_t *adc_hw;
void adc_init();
void adc_gpio_init(int gpio);
void adc_run(bool run);
void adc_fifo_drain();
void adc_select_input(int ch);
void adc_set_round_robin(uint32_t mask);
void adc_fifo_setup(bool en, bool dreq, int thresh, bool err, bool shift);
void adc_set_clkdiv(float div);
