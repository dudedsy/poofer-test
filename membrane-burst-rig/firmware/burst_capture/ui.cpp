// ui.cpp - see ui.h. Wire must already be started on GP12/GP13 by the caller.
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "ui.h"

#define TCA_ADDR   0x20
#define OLED_ADDR  0x3C
#define PIN_UP     3
#define PIN_DN     4
#define PIN_SET    5

#define P0_VBAT_EN 0x40
#define P0_RES_N   0x80

static bool tca_ok = false;
static uint8_t out0 = 0x3F, out1 = 0xFF;      // LEDs off (high), VBAT off, panel in reset
static LedColor led_state[4] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF};

static bool oled_awake = false;
static bool oled_need_init = true;
static uint32_t vbat_off_ms = 0;
static uint32_t last_refresh_ms = 0;
static uint8_t shadow[GFX_W * GFX_H / 8];
static bool shadow_valid = false;
static Gfx last_frame;                         // re-sent on wake / refresh
static bool have_frame = false;

// ---------------- I2C helpers ----------------
static bool tca_write(uint8_t reg, uint8_t a, uint8_t b) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(reg); Wire.write(a); Wire.write(b);
  return Wire.endTransmission() == 0;
}
static bool tca_outputs() { return tca_ok && tca_write(0x02, out0, out1); }

static bool oled_cmds(const uint8_t *c, size_t n) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00);                            // Co=0, D/C#=0: command stream
  Wire.write(c, n);
  return Wire.endTransmission() == 0;
}

// Panel register setup from the HS96F04 internal-DC/DC example (UI spec §9 step 5),
// plus internal IREF (R11 is DNP on rev 8 boards) and horizontal addressing mode.
// 0xAD 0x10 = internal IREF, the value Zephyr's SSD1306/1315 driver uses for
// use-internal-iref. 0x30 is the brighter internal setting if this is too dim.
static const uint8_t OLED_SETUP[] = {
  0xD5, 0x90, 0x81, 0xB0, 0xD9, 0x22, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xA1, 0xC8,
  0xDA, 0x12, 0xDB, 0x30, 0xA6, 0xA4, 0xAD, 0x10, 0x20, 0x00, 0x8D, 0x14,
};

static bool oled_power_up() {
  if (!tca_ok) return false;
  // Steps 3-4: reset pulse, then VBAT on (respecting 50 ms minimum off time).
  out0 &= ~(P0_RES_N | P0_VBAT_EN);
  tca_outputs();
  uint32_t since_off = millis() - vbat_off_ms;
  delay(since_off < 50 ? 50 - since_off : 1);
  out0 |= P0_RES_N;
  tca_outputs();
  out0 |= P0_VBAT_EN;
  tca_outputs();
  delay(100);
  static const uint8_t off = 0xAE;
  if (!oled_cmds(&off, 1) || !oled_cmds(OLED_SETUP, sizeof OLED_SETUP)) return false;
  shadow_valid = false;
  oled_need_init = false;
  oled_awake = true;
  last_refresh_ms = millis();
  return true;
}

static void push_frame(const Gfx &g, bool force) {
  for (int page = 0; page < 8; page++) {
    const uint8_t *src = &g.fb[page * GFX_W];
    if (!force && shadow_valid && memcmp(src, &shadow[page * GFX_W], GFX_W) == 0) continue;
    uint8_t addr[] = {0x21, 0, 127, 0x22, (uint8_t)page, (uint8_t)page};
    if (!oled_cmds(addr, sizeof addr)) { oled_need_init = true; return; }
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x40);                          // data stream
    Wire.write(src, GFX_W);
    if (Wire.endTransmission() != 0) { oled_need_init = true; return; }
    memcpy(&shadow[page * GFX_W], src, GFX_W);
  }
  shadow_valid = true;
}

// ---------------- public ----------------
bool ui_begin() {
  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DN, INPUT_PULLUP);
  pinMode(PIN_SET, INPUT_PULLUP);
  // Steps 1-2: output registers first (all LEDs off, VBAT off, panel in reset),
  // then make P0.0-P0.7 and P1.0-P1.5 outputs. Spares P1.6/P1.7 stay inputs.
  out0 = 0x3F; out1 = 0xFF;
  tca_ok = tca_write(0x02, out0, out1) && tca_write(0x06, 0x00, 0xC0);
  if (!tca_ok) return false;
  delay(2);
  if (oled_power_up()) {
    Gfx blank; blank.clear();
    push_frame(blank, true);
    static const uint8_t on = 0xAF;
    oled_cmds(&on, 1);
    delay(100);
  }
  return true;
}

bool ui_present() { return tca_ok; }

void ui_set_led(int n, LedColor c) {
  if (n < 0 || n > 3 || led_state[n] == c) return;
  led_state[n] = c;
  uint8_t on = (uint8_t)c & 7;                 // active low: clear bits that are on
  uint8_t bits = (uint8_t)(~on & 7);
  if (n < 2) { int sh = n * 3; out0 = (out0 & ~(7 << sh)) | (bits << sh); }
  else       { int sh = (n - 2) * 3; out1 = (out1 & ~(7 << sh)) | (bits << sh); }
  tca_outputs();
}

void ui_show(const Gfx &g) {
  last_frame = g;
  have_frame = true;
  if (!tca_ok) return;
  if (!oled_awake || oled_need_init) {
    if (!oled_power_up()) return;
    push_frame(g, true);
    static const uint8_t on = 0xAF;
    oled_cmds(&on, 1);
    return;
  }
  push_frame(g, false);
}

void ui_blank() {
  if (!tca_ok || !oled_awake) return;
  static const uint8_t off[] = {0xAE, 0x8D, 0x10};
  oled_cmds(off, sizeof off);
  out0 &= ~P0_VBAT_EN;
  tca_outputs();
  vbat_off_ms = millis();
  oled_awake = false;
}

bool ui_blanked() { return tca_ok && !oled_awake; }

void ui_service() {
  if (!tca_ok || !oled_awake) return;
  // The panel datasheet recommends periodically re-sending commands and data so
  // noise can't leave the controller in a bad state.
  if (oled_need_init || millis() - last_refresh_ms > 5000) {
    last_refresh_ms = millis();
    if (oled_need_init || !oled_cmds(OLED_SETUP, sizeof OLED_SETUP)) {
      oled_awake = false;                      // full power-up on next ui_show()
      if (have_frame) ui_show(last_frame);
      return;
    }
    static const uint8_t on = 0xAF;
    oled_cmds(&on, 1);
    if (have_frame) push_frame(last_frame, true);
  }
}

// ---------------- buttons ----------------
struct Btn {
  uint8_t pin;
  bool down;           // debounced state
  bool raw;
  uint32_t t_raw;      // when raw last changed
  uint32_t t_down;     // when debounced press began
  uint32_t t_rep;      // next auto-repeat time
  bool long_done;
};
static Btn btns[3] = {{PIN_UP, false, false, 0, 0, 0, false}, {PIN_DN, false, false, 0, 0, 0, false},
                      {PIN_SET, false, false, 0, 0, 0, false}};
static uint32_t last_activity = 0;

uint8_t ui_buttons() {
  uint8_t ev = 0;
  uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    Btn &b = btns[i];
    bool r = digitalRead(b.pin) == LOW;
    if (r != b.raw) { b.raw = r; b.t_raw = now; }
    if (r != b.down && now - b.t_raw >= 15) {          // stable 15 ms
      b.down = r;
      if (r) {
        b.t_down = now; b.long_done = false; b.t_rep = now + 500;
        if (i == 0) ev |= EV_UP;
        if (i == 1) ev |= EV_DN;
      } else if (i == 2 && !b.long_done) {
        ev |= EV_SET;                                   // SET acts on release
      }
    }
    if (b.down) {
      if (i < 2 && (int32_t)(now - b.t_rep) >= 0) {   // UP/DN auto-repeat
        b.t_rep = now + 150;
        ev |= (i == 0) ? EV_UP : EV_DN;
      }
      if (i == 2 && !b.long_done && now - b.t_down >= 800) {
        b.long_done = true;
        ev |= EV_SET_LONG;
      }
    }
  }
  if (ev) last_activity = now;
  return ev;
}

uint32_t ui_last_activity_ms() { return last_activity; }
