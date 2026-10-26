// ui.h - UI (lid) board: TCA9555 expander, SSD1315 OLED, RGB LEDs, and the three
// buttons on the base board's J2 (GP3/GP4/GP5). Follows UI Board Spec Rev A §9.
#pragma once
#include <stdint.h>
#include "gfx.h"

enum LedColor : uint8_t {        // bit 0 red, bit 1 green, bit 2 blue
  LED_OFF = 0, LED_RED = 1, LED_GREEN = 2, LED_AMBER = 3, LED_BLUE = 4,
  LED_MAGENTA = 5, LED_CYAN = 6, LED_WHITE = 7
};

// Button events from ui_buttons(). UP/DN auto-repeat while held.
enum : uint8_t { EV_UP = 1, EV_DN = 2, EV_SET = 4, EV_SET_LONG = 8 };

bool ui_begin();                      // true if the expander answered at 0x20
bool ui_present();
void ui_set_led(int n, LedColor c);   // n = 0..3 (LED1..LED4)
void ui_show(const Gfx &g);           // pushes changed pages; wakes the panel if blanked
void ui_blank();                      // panel off (burn-in), ui_show() wakes it
void ui_reset_display();              // full panel restart (VBAT off, reset, init) on the next ui_show()
bool ui_blanked();
void ui_service();                    // periodic refresh / re-init, call from loop()
uint8_t ui_buttons();                 // debounced events since last call
uint32_t ui_last_activity_ms();       // millis() of the last button event
