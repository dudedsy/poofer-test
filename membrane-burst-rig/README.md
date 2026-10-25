# Membrane burst rig

A small pressurized chamber sealed by a thin membrane. Popping the membrane drops the pressure faster than any of the sensors can follow, so the recorded traces show each sensor's own response time. The goal is to know which pressure sensors can be trusted for the nozzle cold tests, and at which supply voltage.

**Status:** the firmware compiles (arduino-pico core 6.3.1, 3- and 4-channel builds) but has not been run on hardware. The Python tools are tested on synthetic data only. Everything here is a rewrite from the original readme; the first version's source was lost.

## What it measures

- **Each sensor's response time.** This is modeled as a first-order time constant, tau, plus a small delay relative to a reference sensor.
- **Whether sensors behave the same at 3.3 V and at 5 V.** Run one sensor at each voltage side by side.
- **Clipping, ringing, overshoot, and resets.** These show up in the plots and residuals.

The sensors under test are the generic 0-100 psi, 0.5-4.5 V ratiometric transducers (ceramic element, 1/8" NPT, about $16 each). The listings claim at most 3 ms response, and they likely have a snubber in the port. A sensor useful for plenum transients needs roughly 0.2 ms or better; about 1 ms is fine for fill and steady-state pressure.

## Rig hardware

All 1/2" black iron (or brass) pipe fittings.

| Part | Role |
|---|---|
| 1/2" cross | The chamber |
| Front port | Union holding the membrane between its faces. Use a flat-face gasketed union, or check that a ground-joint union doesn't crease the membrane. The outlet half is cut back past the nut so the gas vents straight out and a pin can reach the membrane. |
| Both side ports | 1/2" x 1/8" NPT bushings, one sensor each. Opposite ports give both sensors the same path. |
| Back port | Tee: fill needle valve on one side, and on the other side either a sensor (3rd channel) or a capped nipple that adds a known volume. |

**Measure the real numbers, don't trust these.** Find the volume by filling everything that sees pressure with water from a syringe, with the sensors installed. Use the union's bore as the opening diameter: 1/2" Sch 40 ID is about 0.622" (15.8 mm).

**Expected event timing** (moderate confidence):

| Stage | Time |
|---|---|
| Membrane tearing open | about 0.1 to 1 ms |
| Expansion wave reaching the sensors | about 0.1 ms |
| Chamber blowdown | time constant roughly 0.4 to 0.7 ms |

That's comfortably faster than a 3 ms sensor, so a 3 ms lag will be obvious. The rig can't resolve sensors much faster than about 0.2 ms.

**Membranes.**
- Thin Mylar (0.5 to 1 mil) or a patch of balloon latex clamped across the bore.
- At 30 psi on a 1/2" bore the load is about 9 lbf, so both materials are feasible.
- Avoid aluminum foil: it tears progressively, which gives a slow, messy opening.

**Two ways to burst:**
- **Pin:** pressurize, then pierce. Use a long pin or a pin on a stick.
- **Over-pressure (cleaner):** fill slowly through the needle valve until the membrane bursts on its own. Set the burst pressure with membrane thickness or the number of layers.

**Chamber ringing.** The chamber has its own acoustic modes in the kHz range. They appear identically on all sensors, so they belong to the chamber, not the sensors.

## Electronics

RP2040-Zero, running sketch `firmware/burst_capture/burst_capture.ino`.

| Signal | RP2040-Zero pin |
|---|---|
| Sensor A | GP26 (ADC0) |
| Sensor B | GP27 (ADC1) |
| Sensor C | GP28 (ADC2) |
| 5 V rail divider (5 V mode only) | GP29 (ADC3) |

### 3.3 V supply

Use this only if the static gauge check shows the sensor is linear and stable at 3.3 V.

- Sensor + to 3V3, - to GND.
- Signal through 1 kΩ to the ADC pin, with 1 nF from the pin to GND.
- No rail channel is needed: the sensor is ratiometric to the same 3V3 that feeds ADC_VREF (through R6 on the Zero).
- Running below the 5 V spec can cause problems that only appear in dynamic tests:
  - clipping on overshoot,
  - supply noise passing through,
  - brownout resets on supply dips,
  - slower response.
- Compare one sensor at 3.3 V against one at 5 V in the same burst before trusting 3.3 V.

### 5 V supply

- Sensor + to 5 V.
- Signal through a 10k / 15k divider, with 1 nF at the pin (about 26 kHz corner).
- Put the same divider from 5 V to GP29.
- Use matched or 0.1% resistors.
- In the sketch, set `N_CH 4` and `RAIL_CH 3`.
- In the analysis config, set `"supply_mode": "rail"` and `"rail_ch": 3`.

**Keep the filter caps small.** A 100 nF cap would put about 1 ms of lag in front of the sensor and contaminate the very thing being measured.

**Power the sensors from a clean supply,** separate from anything switching heavy loads.

## Firmware

Setup:
1. In Arduino IDE, add the board URL `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`.
2. Install "Raspberry Pi Pico/RP2040/RP2350".
3. Select **Waveshare RP2040 Zero**.

Behavior:
- 300 ksps total, round-robin over the channels (100 kHz per channel with 3 channels, 75 kHz with 4).
- DMA streams into a 192 KB ring buffer, about 320 ms deep.
- Between captures, it streams a 1 kHz feed of 1 ms means.
- **Trigger:** any sensor's 1 ms mean drops more than `trig_drop` counts (default 30, about 1 psi at 3.3 V) below its maximum over the previous 20 ms.
- **Capture window:** 50 ms before the trigger and 200 ms after. The ADC pauses while the capture is sent to the laptop, then restarts and re-arms after 300 ms.

Serial commands:

| Command | Action |
|---|---|
| `t` | Force a capture now. The burst has to happen within about 200 ms, so lowering the threshold is usually easier. |
| `d<N>` | Set the trigger threshold, e.g. `d15`. |
| `?` | Print the JSON header. |

## Logging

Install:
```
pip install -r requirements.txt
```

Run:
```
python host/burst_logger.py COM5 --label mylar_base --notes "base volume, 3.3V"
```

- Each run gets its own folder under `runs/`, holding:
  - `run.json`
  - `slow.csv`
  - `capture_NNN.npz` (raw 12-bit counts, shape frames x channels)
  - `capture_NNN.json` (sample rate, trigger index, flags)
- The console prints each capture's pre-burst and settled counts, and flags ADC errors or framing errors.
- While it runs, type `t`, `d<N>`, `?`, or `q` then Enter.

## Analysis

```
python analysis/blowdown_fit.py rig_config.json --plots plots --per-capture
```

Steps:
1. Measure the chamber volume for each configuration (water and syringe) and the opening bore.
2. Copy `analysis/rig_config.example.json` to `rig_config.json` and fill it in:
   - gas,
   - ambient pressure (about 87 kPa in Reno, about 101 kPa at sea level),
   - one `groups` entry per volume and membrane type, matched to run labels by a glob.
3. Record the vented zero counts for each sensor. This is optional and only used as a sanity check.

**Model.** Lumped adiabatic blowdown through the opening:
- Choked flow, then subsonic flow.
- The opening area ramps linearly over `t_open` from the burst time.
- The chamber pressure is solved exactly from a table, so fits are fast.
- Each sensor sees that pressure, shifted by a relative delay, through a first-order lag.

**Calibration.** Each sensor's own pre-burst and settled readings serve as its 1 and 0. So offset and gain errors don't bias tau.

**Fitted parameters:**
- Cd per membrane type,
- tau and relative delay per sensor,
- burst time and opening time per capture.

`--per-capture` refits each capture alone and plots tau against burst pressure.

### Getting trustworthy results

- **Use two volumes** (add the capped nipple) and a spread of burst pressures, about 10 to 60 psig. With one volume, blowdown time and sensor lag trade off, and the script warns about it.
- **About 6 or more bursts per volume** per sensor configuration.
- **Trust the per-capture scatter, not the joint fit's +/- values.** Those assume the model is exact and will be far too small on real data.
- **Read the residual plots.** Systematic structure means one of three things:
  - acoustic ringing (shared by all sensors),
  - a sensor that isn't first-order (overshoot, or steps from its conditioning chip's update rate),
  - a non-linear membrane opening.
- **Test the sensor, not the plumbing.** Mount each sensor as close to the chamber as possible, with minimal dead volume.

### Synthetic check

Truth: tau = 0.5 / 1.5 / 4.0 ms, delays 0 / 0.05 / -0.03 ms, Cd 0.72. The synthetic pressure is integrated independently of the fitter, with per-sensor gain and offset errors, noise, and 12-bit quantization.

| Case | Joint fit | Per-capture medians |
|---|---|---|
| Exact model | within 0.1% | within 0.1% |
| ±8% Cd scatter per capture + 5% acoustic ring | within 2% (0.491 / 1.498 / 4.000 ms) | within 0.1% |

This demonstrates the code and that the parameters are identifiable. It says nothing about real-world accuracy.

```
python analysis/make_synthetic.py syn --mismatch
python analysis/blowdown_fit.py syn/rig_config.json --plots syn/plots --per-capture
python tests/test_parser.py
```

## Safety

- Face shield and hearing protection.
- Point the open end away from people.
- Start at 10 to 20 psi and work up.
- Shop air is fine for all of these tests.
