# Board bring-up test

Firmware for bench-testing the production boards: the DMX base board (Rev A), the UI lid board (Rev A), and, when stacked, the solenoid driver and igniter (spark) daughterboards. It runs a full test sequence at boot, prints a graded report over USB serial, and then takes single-letter commands to rerun individual tests.

**Status:** run on hardware. A base board with UI board and OLED passes the full sequence on 24 V (all status LEDs green). The solenoid test (`v`) and igniter test (`g`) have been run on one board each. DMX receive is not tested yet (no DMX source on the bench).

**Bench only.** The full run drives the CH1-CH6 gate lines and, if a solenoid board is detected, opens its valve. Keep the gas closed and nothing connected to a burner. An igniter wired to the selected channel will spark during the CH walk.

## Building

- Arduino IDE with the Earle Philhower **Raspberry Pi Pico/RP2040** core. Built against core 6.3.0.
- Tools > Board: **Waveshare RP2040 Zero**.
- Library: **Adafruit NeoPixel** (Library Manager), for the module's WS2812 on GP16.
- Open `firmware/main_board_test/main_board_test.ino`. First upload goes to the UF2 port (hold BOOT while plugging in); after that the board shows up as a normal serial port.
- Serial Monitor at any baud (USB). Commands are single letters; line endings are ignored.

## Power

Most of the base board tests run on USB alone. +5V comes only from the U1 buck off 24 V (D2 stops the module's USB 5 V from back-feeding it), so on USB alone the UI LEDs, the OLED's VBAT, the DMX transceiver and the solenoid gate driver are unpowered. The firmware detects this and runs those parts electrically only, or skips them. Connect 24 V for the full visual pass.

## Full sequence (`a`, also runs at boot)

| Step | Checks |
|---|---|
| Boot info | Module unique ID, reset cause, core temperature |
| WS2812 | Red, green, blue, white (visual) |
| +5V / DMX logic side | +3V3_DMX present via R38/R39 (infers 24 V and U1, U5 alive); R37 DE pulldown |
| I2C lines | Both lines pulled up by R3/R4, not stuck; 9-clock recovery if SDA is held |
| FRAM U4 | R47 WP pull-up, ACK at 0x50, device ID, write protect works, write/readback/restore at 0x1FF0, no address aliasing |
| UI board U1 (TCA9555, 0x20) | Power-on state (R10, R13 pulldowns), register identity, all 12 LED sink lines read back; visual LED walk on 24 V |
| OLED (SSD1315, 0x3C) | Held in reset until U1 releases it; init with internal IREF; white, checkerboard, orientation pattern on 24 V |
| Address report | Every expected and possible address on the bus (base, UI, igniter), found / missing / unexpected |
| Buttons | Idle level; press test for UP, DN, SET with bounce and cross-talk check (UI board present) |
| ADC0-3 | Readings (floating with nothing stacked) |
| SPARE1 | Which pull, if any, is fitted |
| CH1-CH6 | R11-R16 pulldowns, line-to-line isolation, 0.5 s walk per channel |
| Igniter board | If anything answers at 0x48-0x4F: ADS1115 and TMP1075/LM75B identity, cold-junction temperature, thermocouple reading (pegged full scale = open probe, expected with nothing on J4). Output is not toggled; LED2 (OUT) blinks during the CH walk |
| Solenoid board | If an ADC line is held low by ADC_SENSE (needs the JP2 shunt): T1-T3 and T5 from the solenoid spec on JP1 pair 1, 24 V coil, no T_max. No current is a WARN here (no valve fitted yet is normal). Skipped without 24 V |
| Summary | Graded PASS / WARN / FAIL / INFO / SKIP, grouped by board |

Status LEDs at the end: module WS2812 shows the overall result; on 24 V the UI LEDs show base, UI, OLED and overall. Green pass, yellow warn, red fail, blue nothing graded.

## Commands

| Key | Does |
|---|---|
| `a` | Full sequence |
| `s` | I2C line check, scan and address report |
| `f` | FRAM test |
| `u` | UI board (U1 and LED walk) |
| `o` | OLED test |
| `b` | Button press test, runs even if no UI board was detected |
| `c` | CH1-CH6 checks and walk |
| `w` | 20 kHz PWM on CH1-CH6 at 10-60 % for scoping. Asks first; not with a solenoid board stacked |
| `x` | ADC monitor |
| `l` | WS2812 colour cycle |
| `p` | +5V rail / DE pulldown check |
| `i` | I2C diagnostic: tries 0x50-0x57 with two methods at 100 and 400 kHz |
| `t` | Slow open-drain SDA/SCL toggle for metering; module LED shows amber (neither low), purple (SDA low), cyan (SCL low) |
| `v` | Solenoid board, interactive: asks JP1 pair, JP2 line, coil type; optional T_max measurement |
| `g` | Igniter board, interactive: chip checks, thermocouple, then toggles the JP1 channel (default 6) 3 s on / 3 s off with I2C error count |
| `m` | Live thermocouple monitor |
| `k` | Toggle functional I2C clock 100 / 400 kHz |
| `r` | Reprint summary |
| `h` | Menu |

## Settings in the sketch

- `WS2812_ORDER`: `NEO_GRB`. If the red step shows green, change to `NEO_RGB`.
- `OLED_IREF`: `0x30` (internal, 30 uA). Required because R11 is DNP on the built UI boards. `0x10` is the dimmer 19 uA setting.
- OLED orientation is set by `0xA1` / `0xC8` in `oledInitSeq()`. On the first board the orientation block showed bottom-right instead of top-left; swap to `0xA0` / `0xC0` once the lid mounting is settled.

## Notes from bring-up

- **Screen the RP2040-Zero modules with this test.** The first module had a damaged GP13: it toggled bare (against the ~50k internal pull-up) but could not pull SCL low against R4 (2.2k), so every I2C device looked absent. The `t` command and a module swap found it. A module that passes the FRAM and I2C checks here has good GP12/GP13 under real load.
- Thermocouple temperatures printed by `g` and `m` use a linear approximation near room temperature (K ~40.7 uV/C, N ~26.5 uV/C). Good for a warm-finger check, not for flame temperatures; production firmware should use the NIST ITS-90 polynomials.
- The igniter board has no current sense, so its output can only be checked by LED2 or a meter (TP2 to TP6 with U1 empty, J3 with the link or buck fitted).
