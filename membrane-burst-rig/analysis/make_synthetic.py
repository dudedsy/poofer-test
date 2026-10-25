#!/usr/bin/env python3
"""make_synthetic.py - fake burst captures with known sensor lags, to check blowdown_fit.py.

Writes two run folders (base volume and base + capped port) in the burst_logger.py format,
plus a rig_config.json pointing at them.

Truth: tau = 0.5 / 1.5 / 4.0 ms, relative delays 0 / +0.05 / -0.03 ms, Cd = 0.72.
The pressure history is integrated directly with solve_ivp (not with the fitter's
tabulated solution), and each sensor gets its own gain and offset error, noise and
12-bit quantization.

--mismatch adds things the fitter's model does not contain:
    +/-8 % Cd scatter per capture, and a 5 % damped acoustic ring (2.5 kHz, 3 ms decay)
    on the chamber pressure after the burst.

Usage
    python make_synthetic.py OUTDIR [--mismatch] [--seed N]
    python blowdown_fit.py OUTDIR/rig_config.json --plots OUTDIR/plots --per-capture
"""
import argparse
import json
import os

import numpy as np
from scipy.integrate import solve_ivp

PSI = 6894.757
GAMMA, R, T0 = 1.400, 287.05, 293.15
PA = 101325.0
TAU = [0.5e-3, 1.5e-3, 4.0e-3]
DELAY = [0.0, 0.05e-3, -0.03e-3]
CD = 0.72
BORE = 12.0e-3
FS = 100000.0
PRE_MS, POST_MS = 20, 80          # matches the firmware capture window
RANGE, LO, HI, FULL = 100.0, 0.1, 0.9, 4095


def pressure_history(p0, V, cd, t_b, t_open, t):
    A = np.pi * BORE ** 2 / 4
    g = GAMMA
    rcrit = (2 / (g + 1)) ** (g / (g - 1))

    def rhs(tt, y):
        p = max(y[0], PA)
        tau = tt - t_b
        a = 0.0 if tau <= 0 else (A if tau >= t_open else A * tau / t_open)
        T = T0 * (p / p0) ** ((g - 1) / g)
        r = PA / p
        if r <= rcrit:
            q = p * np.sqrt(g / (R * T)) * (2 / (g + 1)) ** ((g + 1) / (2 * (g - 1)))
        else:
            q = p * np.sqrt(max(0.0, 2 * g / ((g - 1) * R * T) * (r ** (2 / g) - r ** ((g + 1) / g))))
        return [-g * R * T / V * cd * a * q]

    out = np.full_like(t, p0)
    t_stop = min(t[-1], t_b + 0.06)                 # chamber is long since vented by then
    sel = (t >= t_b) & (t <= t_stop)
    sol = solve_ivp(rhs, (t_b, t_stop), [p0], t_eval=t[sel], max_step=4e-6, rtol=1e-8, atol=1e-3)
    out[sel] = sol.y[0]
    out[t > t_stop] = PA
    return np.maximum(out, PA)


def lag(x, dt, tau):
    """Sensor first-order lag, integrated on a 10x finer grid (independent of the fitter's filter)."""
    n = 10
    xf = np.interp(np.arange(len(x) * n) / n, np.arange(len(x)), x)
    y = np.empty_like(xf)
    yy = xf[0]
    h = dt / n
    a = np.exp(-h / tau)
    for i, v in enumerate(xf):
        yy = a * yy + (1 - a) * v
        y[i] = yy
    return y[::n]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--mismatch", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    frames = (PRE_MS + POST_MS) * int(FS / 1000)
    trig = PRE_MS * int(FS / 1000)
    t = (np.arange(frames) - trig) / FS
    gain_err = 1 + rng.uniform(-0.03, 0.03, 3)
    off_err = rng.uniform(-20, 20, 3)
    zeros = (FULL * LO + off_err).round().tolist()

    groups = []
    for label, vol_cc in (("base", 21.0), ("capped", 61.0)):
        run = os.path.join(a.outdir, "runs", f"synthetic_{label}")
        os.makedirs(run, exist_ok=True)
        for n, p0g in enumerate(np.linspace(12, 60, 7), start=1):
            p0g += rng.uniform(-1, 1)
            p0 = PA + p0g * PSI
            cd = CD * (1 + rng.uniform(-0.08, 0.08)) if a.mismatch else CD
            t_b = rng.uniform(-0.2e-3, 0.6e-3)
            t_open = rng.uniform(0.15e-3, 0.4e-3)
            fine_t = np.arange(t[0], t[-1] + 1e-12, 1e-6)
            p = pressure_history(p0, vol_cc * 1e-6, cd, t_b, t_open, fine_t)
            if a.mismatch:
                tt = fine_t - t_b
                ring = np.where(tt > 0, 0.05 * (p0 - PA) * np.sin(2 * np.pi * 2500 * tt) * np.exp(-tt / 3e-3), 0)
                p = p + ring
            counts = np.zeros((frames, 3), dtype=np.uint16)
            for k in range(3):
                pk = np.interp(t - DELAY[k], fine_t, p)
                pk_s = lag(pk, 1 / FS, TAU[k])
                v = FULL * (LO + (HI - LO) * (pk_s - PA) / PSI / RANGE) * gain_err[k] + off_err[k]
                v = v + rng.normal(0, 1.5, frames)
                counts[:, k] = np.clip(np.round(v), 0, FULL)
            base = os.path.join(run, f"capture_{n:03d}")
            np.savez_compressed(base + ".npz", counts=counts)
            json.dump(dict(number=n, frames=frames, n_ch=3, trig_frame=trig, fs_ch=FS, forced=False,
                           err_samples=0, framing_ok=True, label=f"synthetic_{label}",
                           notes="synthetic", rail_ch=-1, truth=dict(p0_psig=p0g, Cd=cd, t_b=t_b,
                                                                     t_open=t_open)),
                      open(base + ".json", "w"), indent=2)
        groups.append(dict(captures=f"runs/synthetic_{label}/capture_*.npz", volume_cc=vol_cc,
                           bore_mm=BORE * 1e3, membrane="mylar"))

    cfg = dict(gas="air", T0_K=T0, p_amb_kPa=PA / 1000, supply_mode="fixed",
               sensor=dict(range_psi=RANGE, out_lo_frac=LO, out_hi_frac=HI, adc_full=FULL),
               sensors=[dict(name=n, ch=k, zero=zeros[k]) for k, n in enumerate("ABC")],
               ref_sensor="A", groups=groups,
               fit=dict(pre_ms=[-18, -5], end_ms=[50, 78], window_ms=[-3, 30]),
               truth=dict(tau_ms=[x * 1e3 for x in TAU], delay_ms=[x * 1e3 for x in DELAY], Cd=CD,
                          mismatch=a.mismatch))
    with open(os.path.join(a.outdir, "rig_config.json"), "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"wrote {a.outdir}/rig_config.json and 14 captures (mismatch={a.mismatch})")


if __name__ == "__main__":
    main()
