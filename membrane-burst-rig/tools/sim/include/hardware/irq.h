#pragma once
#define DMA_IRQ_1 12
#define PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY 0x80
typedef void (*irq_handler_t)();
void irq_add_shared_handler(int num, irq_handler_t h, int prio);
void irq_set_enabled(int num, bool en);
