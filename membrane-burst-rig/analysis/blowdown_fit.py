#!/usr/bin/env python3
"""blowdown_fit.py - recover pressure-sensor lag from membrane burst captures.

Physics model (per capture)
    Lumped adiabatic blowdown of a rigid volume V through an opening of bore D:
        dp/dt = -(gamma R T(p) / V) * Cd * A(t) * q(p)
    q(p) is the ideal-gas mass flux per unit area, choked while p/pa > critical ratio and
    subsonic after; T(p) = T0 (p/p0)^((gamma-1)/gamma). The opening area ramps linearly
    from 0 to pi D^2/4 over t_open starting at the burst time t_b.
    Because A(t) and p separate, the solution is exact: Phi(p) = (Cd A/V) * integral(ramp),
    with Phi tabulated once per capture.

Sensor model
    Each sensor sees the normalized pressure (p - pa)/(p0 - pa) delayed by d_s, through a
    first-order lag with time constant tau_s. The reference sensor's delay is fixed at 0,
    so other delays are relative.

Calibration
    Each sensor's own mean before the burst and its settled mean at the end of the capture
    define 1 and 0 of its normalized signal, so sensor gain/offset errors do not bias tau.
    The gauge pressure p0 used by the model comes from those readings and the nominal
    sensor transfer function (only the choked/subsonic split depends on it).

Fitted parameters
    joint fit: Cd per membrane type, tau and relative delay per sensor, t_b and t_open per capture
    --per-capture: everything refit for each capture alone (Cd per capture), for a scatter
    that shows the real spread. The joint fit's +/- assume the model is exact; trust the
    per-capture spread instead.

Usage
    python blowdown_fit.py rig_config.json --plots plots --per-capture
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter
from scipy.sparse import lil_matrix

GASES = {  # gamma, R [J/kg/K]
    "air": (1.400, 287.05),
    "n2": (1.400, 296.80),
    "co2": (1.289, 188.92),
    "ar": (1.667, 208.13),
}
PSI = 6894.757


# ---------------------------------------------------------------- physics
class Blowdown:
    """Normalized pressure history for one capture; cheap to evaluate for any Cd, t_b, t_open."""

    def __init__(self, p0, pa, V, D, gamma, R, T0, n=4000):
        self.p0, self.pa, self.V = p0, pa, V
        self.A = np.pi * D * D / 4
        g = gamma
        s = np.linspace(0.0, 1.0, n)
        x = (p0 - pa) * s * s                       # p - pa, dense near pa (sqrt singularity)
        p = pa + x
        T = T0 * (p / p0) ** ((g - 1) / g)
        r = pa / p
        rcrit = (2 / (g + 1)) ** (g / (g - 1))
        q = np.where(
            r <= rcrit,
            p * np.sqrt(g / (R * T)) * (2 / (g + 1)) ** ((g + 1) / (2 * (g - 1))),
            p * np.sqrt(np.clip(2 * g / ((g - 1) * R * T) * (r ** (2 / g) - r ** ((g + 1) / g)), 0, None)),
        )
        H = g * R * T * q                           # dp/dt = -(Cd A(t)/V) * H(p)
        # Phi(p) = integral_p^p0 dp'/H. Change variable x = X s^2: dx = 2 X s ds; H ~ sqrt(x) near 0.
        integrand = np.empty_like(s)
        integrand[1:] = 2 * (p0 - pa) * s[1:] / H[1:]
        integrand[0] = 2 * integrand[1] - integrand[2]  # finite limit, extrapolate
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (integrand[1:] + integrand[:-1]) * np.diff(s))])
        phi = cum[-1] - cum                         # Phi at each grid point (0 at p0)
        self._phi = phi[::-1]                       # increasing
        self._yn = (x / (p0 - pa))[::-1]            # normalized pressure, 1 -> 0
        self.phi_max = phi[0]

    def yn(self, t, Cd, t_b, t_open):
        tau = np.asarray(t) - t_b
        t_open = max(t_open, 1e-9)
        S = np.where(tau <= 0, 0.0, np.where(tau < t_open, tau * tau / (2 * t_open), tau - t_open / 2))
        phi = Cd * self.A / self.V * S
        return np.interp(phi, self._phi, self._yn, left=1.0, right=0.0)


def first_order(x, dt, tau, x0=1.0):
    """Exact response of 1/(tau s + 1) to piecewise-linear input sampled at dt, starting settled at x0."""
    a = np.exp(-dt / tau)
    k = tau / dt * (1 - a)
    b = [1 - k, k - a]
    zi = [x0 * k]                    # steady state y = x = x0: z = b1*x0 + a*x0 = k*x0
    y, _ = lfilter(b, [1.0, -a], x, zi=zi)
    return y


# ---------------------------------------------------------------- data
def load_config(path):
    with open(path) as f:
        cfg = json.load(f)
    cfg["_dir"] = os.path.dirname(os.path.abspath(path))
    return cfg


def gas_props(cfg):
    g = cfg.get("gas", "air")
    if isinstance(g, dict):
        return g["gamma"], g["R"]
    return GASES[g.lower()]


def load_captures(cfg):
    sc = cfg.get("sensor", {})
    rng = sc.get("range_psi", 100.0)
    lo, hi = sc.get("out_lo_frac", 0.1), sc.get("out_hi_frac", 0.9)
    full = sc.get("adc_full", 4095)
    fitc = cfg.get("fit", {})
    pre_ms = fitc.get("pre_ms", [-18, -5])
    end_ms = fitc.get("end_ms", [50, 78])
    rail_mode = cfg.get("supply_mode", "fixed") == "rail"
    rail_ch = cfg.get("rail_ch")
    sensors = cfg["sensors"]
    pa = cfg.get("p_amb_kPa", 101.325) * 1000

    caps = []
    for gi, grp in enumerate(cfg["groups"]):
        pattern = grp["captures"]
        if not os.path.isabs(pattern):
            pattern = os.path.join(cfg["_dir"], pattern)
        files = sorted(glob.glob(pattern))
        if not files:
            print(f"warning: no captures match {grp['captures']}", file=sys.stderr)
        for fn in files:
            meta = json.load(open(fn[:-4] + ".json"))
            if not meta.get("framing_ok", True):
                print(f"skipping {fn}: framing error", file=sys.stderr)
                continue
            counts = np.load(fn)["counts"].astype(float)
            fs = float(meta["fs_ch"])
            t = (np.arange(counts.shape[0]) - meta["trig_frame"]) / fs
            sig = []
            if rail_mode:
                w = max(1, int(fs / 1000))
                rail = np.convolve(counts[:, rail_ch], np.ones(w) / w, mode="same")
            for s in sensors:
                c = counts[:, s["ch"]]
                if rail_mode:
                    c = c / rail
                sig.append(c)
            sig = np.array(sig)
            pre = (t >= pre_ms[0] / 1e3) & (t < pre_ms[1] / 1e3)
            end = (t >= end_ms[0] / 1e3) & (t < end_ms[1] / 1e3)
            if pre.sum() < 10 or end.sum() < 10:
                sys.exit(f"{fn}: capture spans {t[0]*1e3:.0f} to {t[-1]*1e3:.0f} ms, but fit.pre_ms={pre_ms} / "
                         f"fit.end_ms={end_ms} select {pre.sum()} / {end.sum()} samples. Adjust them in the config.")
            c_pre, c_end = sig[:, pre].mean(1), sig[:, end].mean(1)
            sens = (hi - lo) / rng * (1.0 if rail_mode else full)   # per psi
            p0g_each = (c_pre - c_end) / sens
            for k, s in enumerate(sensors):
                z = s.get("zero")
                if z is not None and abs(c_end[k] - z) > 0.03 * (hi - lo) * (1.0 if rail_mode else full):
                    print(f"note: {os.path.basename(fn)} {s['name']}: settled {c_end[k]:.1f} vs zero {z}"
                          " (not fully vented in the window?)", file=sys.stderr)
            yn = (sig - c_end[:, None]) / (c_pre - c_end)[:, None]
            noise = sig[:, pre].std(1) / np.abs(c_pre - c_end)
            caps.append(dict(file=fn, name=os.path.relpath(fn, cfg["_dir"]), group=gi,
                             membrane=grp.get("membrane", f"group{gi}"),
                             V=grp["volume_cc"] * 1e-6, D=grp["bore_mm"] * 1e-3,
                             t=t, fs=fs, y=yn, p0g_psi=float(np.mean(p0g_each)),
                             p0g_each=p0g_each, noise=noise, pa=pa))
    return caps


def initial_tb(cap, ref):
    y = cap["y"][ref]
    w = max(1, int(cap["fs"] / 5000))
    ys = np.convolve(y, np.ones(w) / w, mode="same")
    # skip the smoothing's edge (mode="same" pads with zeros) and the pre-trigger
    # stretch that a short capture starts with
    t = cap["t"]
    idx = np.where((t > max(-0.02, t[min(len(t) - 1, 2 * w)])) & (ys < 0.97))[0]
    return cap["t"][idx[0]] - 0.0005 if len(idx) else 0.0


# ---------------------------------------------------------------- fitting
class Problem:
    def __init__(self, caps, cfg, gamma, R):
        self.caps, self.cfg = caps, cfg
        self.sensors = cfg["sensors"]
        self.ns = len(self.sensors)
        names = [s["name"] for s in self.sensors]
        self.ref = names.index(cfg.get("ref_sensor", names[0]))
        fitc = cfg.get("fit", {})
        win = fitc.get("window_ms", [-3, 30])
        T0 = cfg.get("T0_K", 293.15)
        self.membranes = sorted({c["membrane"] for c in caps})
        for c in caps:
            p0 = c["pa"] + c["p0g_psi"] * PSI
            c["model"] = Blowdown(p0, c["pa"], c["V"], c["D"], gamma, R, T0)
            c["tb0"] = initial_tb(c, self.ref)
            m = (c["t"] >= c["tb0"] + win[0] / 1e3) & (c["t"] <= c["tb0"] + win[1] / 1e3)
            c["mask"] = m
            c["w"] = 1.0 / np.maximum(c["noise"], 1e-4)   # per-sensor weights from pre-burst noise

    def layout(self, per_capture_cd=False):
        nm = len(self.caps) if per_capture_cd else len(self.membranes)
        names = [f"Cd[{m}]" for m in (range(nm) if per_capture_cd else self.membranes)]
        names += [f"tau_ms[{s['name']}]" for s in self.sensors]
        names += [f"delay_ms[{s['name']}]" for k, s in enumerate(self.sensors) if k != self.ref]
        for i, c in enumerate(self.caps):
            names += [f"tb_ms[{i}]", f"topen_ms[{i}]"]
        return names

    def x0_bounds(self, per_capture_cd=False):
        nm = len(self.caps) if per_capture_cd else len(self.membranes)
        x0 = [0.7] * nm + [1.0] * self.ns + [0.0] * (self.ns - 1)
        lb = [0.05] * nm + [0.005] * self.ns + [-3.0] * (self.ns - 1)
        ub = [1.5] * nm + [50.0] * self.ns + [3.0] * (self.ns - 1)
        for c in self.caps:
            x0 += [c["tb0"] * 1e3, 0.3]
            lb += [c["tb0"] * 1e3 - 5, 0.001]
            ub += [c["tb0"] * 1e3 + 5, 10.0]
        return np.array(x0), np.array(lb), np.array(ub)

    def unpack(self, x, per_capture_cd=False):
        nm = len(self.caps) if per_capture_cd else len(self.membranes)
        cd = x[:nm]
        tau = x[nm:nm + self.ns] / 1e3
        d = np.insert(x[nm + self.ns:nm + 2 * self.ns - 1], self.ref, 0.0) / 1e3
        cp = x[nm + 2 * self.ns - 1:].reshape(-1, 2) / 1e3
        return cd, tau, d, cp

    def predict(self, i, cd_val, tau, d, tb, topen, t=None):
        c = self.caps[i]
        t = c["t"] if t is None else t
        out = []
        for k in range(self.ns):
            x = c["model"].yn(t - d[k], cd_val, tb, topen)
            out.append(first_order(x, 1.0 / c["fs"], tau[k]))
        return np.array(out)

    def residuals(self, x, per_capture_cd=False):
        cd, tau, d, cp = self.unpack(x, per_capture_cd)
        res = []
        for i, c in enumerate(self.caps):
            cdv = cd[i] if per_capture_cd else cd[self.membranes.index(c["membrane"])]
            m = c["mask"]
            pred = self.predict(i, cdv, tau, d, cp[i, 0], cp[i, 1], t=c["t"])
            res.append(((pred[:, m] - c["y"][:, m]) * c["w"][:, None]).ravel())
        return np.concatenate(res)

    def sparsity(self, per_capture_cd=False):
        nm = len(self.caps) if per_capture_cd else len(self.membranes)
        nshared = nm + 2 * self.ns - 1
        sizes = [int(c["mask"].sum()) * self.ns for c in self.caps]
        J = lil_matrix((sum(sizes), nshared + 2 * len(self.caps)), dtype=int)
        r0 = 0
        for i, c in enumerate(self.caps):
            rows = slice(r0, r0 + sizes[i])
            if per_capture_cd:
                J[rows, i] = 1
            else:
                J[rows, self.membranes.index(c["membrane"])] = 1
            J[rows, nm:nshared] = 1
            J[rows, nshared + 2 * i: nshared + 2 * i + 2] = 1
            r0 += sizes[i]
        return J

    def fit(self, per_capture_cd=False, verbose=0):
        x0, lb, ub = self.x0_bounds(per_capture_cd)
        x0 = np.clip(x0, lb + 1e-9, ub - 1e-9)
        r = least_squares(self.residuals, x0, bounds=(lb, ub), args=(per_capture_cd,),
                          jac_sparsity=self.sparsity(per_capture_cd), x_scale="jac",
                          method="trf", verbose=verbose, max_nfev=400)
        J = r.jac.toarray() if hasattr(r.jac, "toarray") else r.jac
        dof = max(1, len(r.fun) - len(r.x))
        s2 = 2 * r.cost / dof
        try:
            cov = np.linalg.pinv(J.T @ J) * s2
            err = np.sqrt(np.clip(np.diag(cov), 0, None))
        except np.linalg.LinAlgError:
            err = np.full_like(r.x, np.nan)
        return r, err, s2


# ---------------------------------------------------------------- reporting
def make_plots(prob, x, per_capture_cd, outdir, tag):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(outdir, exist_ok=True)
    cd, tau, d, cp = prob.unpack(x, per_capture_cd)
    for i, c in enumerate(prob.caps):
        cdv = cd[i] if per_capture_cd else cd[prob.membranes.index(c["membrane"])]
        m = c["mask"]
        t = c["t"][m] * 1e3
        pred = prob.predict(i, cdv, tau, d, cp[i, 0], cp[i, 1])[:, m]
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True,
                                     gridspec_kw=dict(height_ratios=[3, 1.3]))
        for k, s in enumerate(prob.sensors):
            l, = a1.plot(t, c["y"][k, m], ".", ms=1.5, alpha=0.5)
            a1.plot(t, pred[k], "-", color=l.get_color(), lw=1.2, label=f"{s['name']}  tau={tau[k]*1e3:.3g} ms")
            a2.plot(t, c["y"][k, m] - pred[k], "-", color=l.get_color(), lw=0.7)
        pres = c["model"].yn(c["t"][m], cdv, cp[i, 0], cp[i, 1])
        a1.plot(t, pres, "k--", lw=1, label="pressure (model)")
        a1.set_ylabel("normalized (p - pa)/(p0 - pa)")
        a1.set_title(f"{c['name']}  p0={c['p0g_psi']:.1f} psig  V={c['V']*1e6:.1f} cc  Cd={cdv:.3f}", fontsize=9)
        a1.legend(fontsize=8)
        a2.axhline(0, color="k", lw=0.5)
        a2.set_ylabel("residual")
        a2.set_xlabel("time from trigger [ms]")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, f"{tag}_cap{i:03d}.png"), dpi=110)
        plt.close(fig)


def per_capture_scatter(prob, rows, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(outdir, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    vols = sorted({r["V_cc"] for r in rows})
    for k, s in enumerate(prob.sensors):
        for vi, v in enumerate(vols):
            sel = [r for r in rows if r["V_cc"] == v]
            axes[0].plot([r["p0_psig"] for r in sel], [r["tau_ms"][k] for r in sel],
                         "os^"[vi % 3], color=f"C{k}", label=f"{s['name']}  V={v:g} cc")
    axes[0].set_xlabel("burst pressure [psig]")
    axes[0].set_ylabel("tau [ms]")
    axes[0].set_yscale("log")
    axes[0].legend(fontsize=7)
    axes[1].plot([r["p0_psig"] for r in rows], [r["Cd"] for r in rows], "o")
    axes[1].set_xlabel("burst pressure [psig]")
    axes[1].set_ylabel("Cd (per capture)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "per_capture_scatter.png"), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config")
    ap.add_argument("--plots", help="folder for plots")
    ap.add_argument("--per-capture", action="store_true", help="also fit every capture alone")
    ap.add_argument("--out", help="results JSON (default: fit_results.json next to config)")
    ap.add_argument("-v", "--verbose", action="count", default=0)
    a = ap.parse_args()

    cfg = load_config(a.config)
    gamma, R = gas_props(cfg)
    caps = load_captures(cfg)
    if not caps:
        sys.exit("no captures loaded")
    prob = Problem(caps, cfg, gamma, R)
    print(f"{len(caps)} captures, {prob.ns} sensors, gas gamma={gamma} R={R}")
    if len({c['V'] for c in caps}) < 2:
        print("WARNING: only one volume. Blowdown time and sensor lag trade off; add captures"
              " with a second known volume.")

    r, err, s2 = prob.fit(False, a.verbose)
    names = prob.layout(False)
    nshared = len(prob.membranes) + 2 * prob.ns - 1
    print(f"\njoint fit: {r.message}  rms weighted residual={np.sqrt(s2):.3g}")
    print("  (+/- assume a perfect model; use the per-capture spread for real uncertainty)")
    for n, v, e in zip(names[:nshared], r.x[:nshared], err[:nshared]):
        print(f"  {n:22s} {v:9.4f} +/- {e:.4f}")
    results = dict(joint={n: [float(v), float(e)] for n, v, e in zip(names, r.x, err)},
                   rms_weighted_residual=float(np.sqrt(s2)))
    if a.plots:
        make_plots(prob, r.x, False, a.plots, "joint")

    if a.per_capture:
        rows = []
        for i, c in enumerate(caps):
            sub = Problem([c], cfg, gamma, R)
            ri, _, _ = sub.fit(True, 0)
            cd, tau, d, cp = sub.unpack(ri.x, True)
            rows.append(dict(capture=c["name"], V_cc=c["V"] * 1e6, p0_psig=c["p0g_psi"], Cd=float(cd[0]),
                             tau_ms=(tau * 1e3).tolist(), delay_ms=(d * 1e3).tolist(),
                             t_open_ms=float(cp[0, 1] * 1e3), ok=bool(ri.success)))
        print("\nper-capture fits:")
        hdr = "  " + "capture".ljust(34) + " V_cc  p0psig    Cd " + "".join(f" tau[{s['name']}]".rjust(11) for s in prob.sensors)
        print(hdr)
        for row in rows:
            print(f"  {row['capture'][-34:]:34s} {row['V_cc']:5.1f} {row['p0_psig']:7.1f} {row['Cd']:5.3f}"
                  + "".join(f"{v:11.3f}" for v in row["tau_ms"]))
        taus = np.array([row["tau_ms"] for row in rows])
        dl = np.array([row["delay_ms"] for row in rows])
        print("\n  sensor      tau median [ms]  (IQR)          delay median [ms]")
        for k, s in enumerate(prob.sensors):
            q1, q2, q3 = np.percentile(taus[:, k], [25, 50, 75])
            print(f"  {s['name']:10s} {q2:8.3f}   ({q1:.3f} - {q3:.3f})   {np.median(dl[:, k]):8.3f}")
        results["per_capture"] = rows
        if a.plots:
            per_capture_scatter(prob, rows, a.plots)

    out = a.out or os.path.join(cfg["_dir"], "fit_results.json")
    with open(out, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nresults written to {out}")


if __name__ == "__main__":
    main()
