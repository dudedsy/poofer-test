#pragma once
#include <stdint.h>
#include <stddef.h>
#define FLASH_SECTOR_SIZE 4096u
#define FLASH_PAGE_SIZE 256u
extern uint8_t sim_flash[2 * 1024 * 1024];
#define XIP_BASE ((uintptr_t)sim_flash)
void flash_range_erase(uint32_t off, size_t len);
void flash_range_program(uint32_t off, const uint8_t *data, size_t len);
