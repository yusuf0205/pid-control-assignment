"""Recreate every graph and table in the report:   python src/make_figures.py
Outputs: figures/*.png and results/results.json"""
import json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

sys.path.insert(0, os.path.dirname(__file__))
from pid import PID
from plants import MassSpringDamper, FirstOrderChain
from sim import simulate, metrics

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(ROOT, "figures"); RES = os.path.join(ROOT, "results")
os.makedirs(FIG, exist_ok=True); os.makedirs(RES, exist_ok=True)
plt.rcParams.update({"figure.dpi": 130, "axes.grid": True, "grid.alpha": 0.3, "font.size": 10,
                     "axes.titlesize": 11, "legend.fontsize": 9})
R = {}                                   # all numbers quoted in the report
FINAL = dict(kp=300.0, ki=200.0, kd=20.0)  # final gains chosen in Section 6


def save(name):
    plt.tight_layout(); plt.savefig(os.path.join(FIG, name), bbox_inches="tight"); plt.close()


def rnd(m):
    return {k: (None if (v is None or (isinstance(v, float) and math.isnan(v))) else round(float(v), 4)) for k, v in m.items()}


# ---------------- Section 1: loop diagram ----------------
def fig_loop():
    fig, ax = plt.subplots(figsize=(8, 2.8)); ax.axis("off"); ax.set_xlim(0, 11.5); ax.set_ylim(-0.2, 3)
    boxes = {"Target\n(setpoint r)": (0.1, 1.7), "Controller\n(PID)": (3.3, 1.7),
             "Machine\n(plant)": (6.5, 1.7), "Sensor": (6.5, 0.0)}
    for txt, (x, y) in boxes.items():
        ax.add_patch(FancyBboxPatch((x, y), 1.8, 0.9, boxstyle="round,pad=0.05", fc="#e8f0fe", ec="#1a4fa0"))
        ax.text(x + 0.9, y + 0.45, txt, ha="center", va="center")
    arr = dict(arrowstyle="->", lw=1.6, color="#333")
    ax.annotate("", (3.3, 2.15), (1.95, 2.15), arrowprops=arr); ax.text(2.62, 2.3, "e = r - y", ha="center", fontsize=8)
    ax.annotate("", (6.5, 2.15), (5.15, 2.15), arrowprops=arr); ax.text(5.82, 2.3, "force u", ha="center", fontsize=8)
    ax.annotate("", (11.3, 2.15), (8.35, 2.15), arrowprops=arr); ax.text(9.8, 2.3, "actual position y", ha="center", fontsize=8)
    ax.plot([10.2, 10.2], [2.15, 0.45], color="#333", lw=1.6); ax.annotate("", (8.35, 0.45), (10.2, 0.45), arrowprops=arr)
    ax.plot([6.5, 1.0], [0.45, 0.45], color="#333", lw=1.6); ax.annotate("", (1.0, 1.65), (1.0, 0.45), arrowprops=arr)
    ax.text(3.7, 0.55, "measured y goes back and is compared with r", fontsize=8, ha="center")
    ax.set_title("Fig 1. The feedback loop: target -> controller -> machine -> sensor -> back to the start")
    save("fig01_loop_diagram.png")


# ---------------- Section 3: the plant on its own ----------------
def fig_open_loop():
    P = MassSpringDamper()
    a = simulate(P, None, 1.0, T=4, open_loop_force=lambda t: 20.0)
    b = simulate(P, None, 1.0, T=4, open_loop_force=lambda t: 20.0 if t < 1.5 else 0.0)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.2))
    ax1.plot(a["t"], a["y"], label="constant push F = 20 N"); ax1.plot(b["t"], b["y"], "--", label="push removed at t = 1.5 s")
    ax1.axhline(1, color="k", lw=0.8, ls=":", label="target 1 m")
    ax1.set(title="Position of the block (no controller)", xlabel="time (s)", ylabel="position x (m)"); ax1.legend()
    ax2.plot(a["t"], a["u"], label="F = 20 N"); ax2.plot(b["t"], b["u"], "--", label="F removed")
    ax2.set(title="Force applied", xlabel="time (s)", ylabel="force F (N)"); ax2.legend()
    fig.suptitle("Fig 2. Holding x = 1 m needs a constant 20 N push (spring force k*x = 20 N/m * 1 m)")
    save("fig02_open_loop.png")
    R["open_loop_final_x"] = round(float(a["y"][-1]), 4)


# ---------------- Section 4: the four measures ----------------
def fig_metrics():
    P = MassSpringDamper(); r = simulate(P, PID(100), 1.0, T=2.0); t, y = r["t"], r["y"]; m = metrics(t, y)
    yf = m["y_final"]; t10 = t[np.argmax(y >= 0.1 * yf)]; t90 = t[np.argmax(y >= 0.9 * yf)]; ip = np.argmax(y)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, y, lw=2, label="position y(t), P-only Kp = 100"); ax.axhline(1, color="k", ls=":", label="target 1 m")
    ax.axhspan(yf * 0.98, yf * 1.02, color="g", alpha=0.15, label="+/-2 % settling band")
    ax.axvspan(t10, t90, color="orange", alpha=0.25, label=f"rise time {m['rise']:.3f} s (10 % to 90 %)")
    ax.annotate(f"overshoot {m['overshoot']:.1f} %", (t[ip], y[ip]), (t[ip] + 0.25, y[ip] + 0.08), arrowprops=dict(arrowstyle="->"))
    ax.axvline(m["settle"], color="g", ls="--", label=f"settling time {m['settle']:.3f} s")
    ax.annotate("", (1.85, 1.0), (1.85, yf), arrowprops=dict(arrowstyle="<->", color="r"))
    ax.text(1.6, (1 + yf) / 2, f"steady-state\nerror {m['sse']:.3f} m", color="r", ha="right", va="center")
    ax.set(title="Fig 3. The four measures marked on one step response", xlabel="time (s)", ylabel="position (m)", ylim=(0, 1.2))
    ax.legend(loc="lower right"); save("fig03_metrics_annotated.png"); R["metrics_example_P100"] = rnd(m)


# ---------------- Section 5: each part alone ----------------
def fig_parts():
    P = MassSpringDamper()
    # P only
    fig, ax = plt.subplots(figsize=(8, 4)); rows = []
    for kp in [10, 50, 100, 300]:
        r = simulate(P, PID(kp), 1.0, T=2.5); m = metrics(r["t"], r["y"])
        wn = math.sqrt(20 + kp); z = 10 / (2 * wn); os_th = 100 * math.exp(-math.pi * z / math.sqrt(1 - z * z))
        th = kp / (20 + kp)
        l, = ax.plot(r["t"], r["y"], label=f"Kp = {kp}: final {m['y_final']:.3f} m (theory {th:.3f})")
        ax.axhline(th, color=l.get_color(), ls=":", lw=0.9)
        rows.append(dict(kp=kp, final_sim=round(m["y_final"], 4), final_theory=round(th, 4),
                         sse_theory=round(20 / (20 + kp), 4), zeta=round(z, 3), os_theory=round(os_th, 2), **{k: rnd(m)[k] for k in ("rise", "overshoot", "settle")}))
    ax.axhline(1, color="k", lw=1, label="target 1 m")
    ax.set(title="Fig 4. P only: higher Kp gets closer but never reaches 1 m, and bounces more", xlabel="time (s)", ylabel="position (m)")
    ax.legend(loc="lower right"); save("fig04_p_only.png"); R["p_only"] = rows

    fig, ax = plt.subplots(figsize=(8, 4)); rows = []
    for ki in [0, 100, 200, 1000]:
        r = simulate(P, PID(300, ki), 1.0, T=12); m = metrics(r["t"], r["y"])
        ax.plot(r["t"], r["y"], label=f"Kp = 300, Ki = {ki}: final {m['y_final']:.3f} m, overshoot {m['overshoot']:.1f} %")
        rows.append(dict(ki=ki, **rnd(m)))
    ax.axhline(1, color="k", lw=1, label="target 1 m"); ax.set_xlim(0, 6)
    ax.set(title="Fig 5. Adding I: the steady-state error disappears (bigger Ki = faster, more overshoot)", xlabel="time (s)", ylabel="position (m)")
    ax.legend(loc="lower right"); save("fig05_add_i.png"); R["pi"] = rows

    fig, ax = plt.subplots(figsize=(8, 4)); rows = []
    for kd in [0, 10, 20, 40]:
        r = simulate(P, PID(300, 0, kd), 1.0, T=1.5); m = metrics(r["t"], r["y"])
        z = (10 + kd) / (2 * math.sqrt(320)); os_th = 100 * math.exp(-math.pi * z / math.sqrt(1 - z * z)) if z < 1 else 0.0
        ax.plot(r["t"], r["y"], label=f"Kd = {kd}: zeta = {z:.2f}, overshoot {m['overshoot']:.1f} % (theory {os_th:.1f} %)")
        rows.append(dict(kd=kd, zeta=round(z, 3), os_theory=round(os_th, 2), **rnd(m)))
    ax.axhline(1, color="k", lw=1, label="target 1 m")
    ax.set(title="Fig 6. Adding D to Kp = 300: damping rises and the bouncing goes away", xlabel="time (s)", ylabel="position (m)")
    ax.legend(loc="lower right"); save("fig06_add_d.png"); R["pd"] = rows


# ---------------- Section 6: all together ----------------
def fig_compare():
    P = MassSpringDamper(); fig, ax = plt.subplots(figsize=(8, 4)); rows = []
    kp, ki, kd = FINAL["kp"], FINAL["ki"], FINAL["kd"]
    for name, g in [("P", (kp, 0, 0)), ("PI", (kp, ki, 0)), ("PD", (kp, 0, kd)), ("PID", (kp, ki, kd))]:
        r = simulate(P, PID(*g), 1.0, T=12); m = metrics(r["t"], r["y"])
        ax.plot(r["t"], r["y"], lw=2 if name == "PID" else 1.3, label=f"{name} (Kp={g[0]:g}, Ki={g[1]:g}, Kd={g[2]:g})")
        rows.append(dict(name=name, kp=g[0], ki=g[1], kd=g[2], **rnd(m)))
    ax.axhline(1, color="k", lw=1, label="target 1 m"); ax.set_xlim(0, 2)
    ax.set(title="Fig 7. P vs PI vs PD vs PID on the test system", xlabel="time (s)", ylabel="position (m)")
    ax.legend(loc="lower right"); save("fig07_compare.png"); R["compare"] = rows
    # small gain search used to justify the final choice
    grid = []
    for kp_ in [100, 200, 300, 400]:
        for ki_ in [100, 200, 400]:
            for kd_ in [10, 20, 30]:
                r = simulate(P, PID(kp_, ki_, kd_), 1.0, T=4); m = metrics(r["t"], r["y"])
                grid.append(dict(kp=kp_, ki=ki_, kd=kd_, **rnd(m), peak_force=round(float(r["u"].max()), 1)))
    R["gain_grid_top"] = sorted(grid, key=lambda d: (d["settle"] or 99))[:6]


# ---------------- Section 7: real-world problems ----------------
def fig_windup():
    P = MassSpringDamper(); fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True); rows = []
    for lab, kw, lim in [("no force limit", {}, None), ("30 N limit, no fix", dict(u_min=-30, u_max=30), 30),
                         ("30 N limit + anti-windup", dict(u_min=-30, u_max=30, anti_windup=True), 30)]:
        c = PID(**FINAL, **kw); r = simulate(P, c, 1.0, T=12); m = metrics(r["t"], r["y"])
        a1.plot(r["t"], r["y"], label=f"{lab}: overshoot {m['overshoot']:.1f} %, settle {m['settle']:.2f} s")
        a2.plot(r["t"], r["u"], label=lab); rows.append(dict(case=lab, **rnd(m)))
    a1.axhline(1, color="k", lw=1); a1.set(title="Fig 8a. Integral windup: position", ylabel="position (m)"); a1.legend(loc="lower right")
    a2.set(title="Fig 8b. Motor force (clipped at 30 N)", xlabel="time (s)", ylabel="force (N)", ylim=(-40, 60), xlim=(0, 5)); a2.legend()
    save("fig08_windup.png"); R["windup"] = rows


def fig_noise():
    P = MassSpringDamper(); fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True); rows = []
    for lab, tau in [("D on raw noisy signal", 0.0), ("D low-pass filtered (tau = 10 ms)", 0.01)]:
        c = PID(**FINAL, d_filter_tau=tau, d_on_measurement=True)
        r = simulate(P, c, 1.0, T=2, noise=0.001); m = metrics(r["t"], r["y"])
        sel = r["t"] > 1.0; ustd = float(np.std(r["u"][sel])); ystd = float(np.std(r["y"][sel])) * 1000
        a2.plot(r["t"], r["u"], lw=0.7, label=f"{lab}: force std {ustd:.1f} N after 1 s")
        a1.plot(r["t"], r["y"], lw=1, label=f"{lab}: position std {ystd:.3f} mm after 1 s")
        rows.append(dict(case=lab, force_std_N=round(ustd, 2), pos_std_mm=round(ystd, 4)))
    a1.axhline(1, color="k", lw=1); a1.set(title="Fig 9a. Sensor noise +/-1 mm: position", ylabel="position (m)", ylim=(0, 1.15)); a1.legend(loc="lower right")
    a2.set(title="Fig 9b. Motor force: raw D turns 1 mm of noise into huge force jumps", xlabel="time (s)", ylabel="force (N)", ylim=(-150, 400)); a2.legend()
    save("fig09_noise.png"); R["noise"] = rows
    R["noise_raw_d_peak_estimate_N"] = FINAL["kd"] * 0.002 / 0.001


def fig_kick():
    P = MassSpringDamper(); sp = lambda t: 0.0 if t < 0.5 else 1.0
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True); rows = []
    for lab, dm in [("D on error", False), ("D on measurement", True)]:
        r = simulate(P, PID(**FINAL, d_on_measurement=dm), sp, T=2)
        a1.plot(r["t"], r["y"], label=lab); a2.plot(r["t"], r["u"], label=f"{lab}: peak force {r['u'].max():.0f} N")
        rows.append(dict(case=lab, peak_force=round(float(r["u"].max()), 1)))
    a1.plot(r["t"], r["r"], "k:", label="target (jumps 0 -> 1 m at 0.5 s)")
    a1.set(title="Fig 10a. Derivative kick: position", ylabel="position (m)"); a1.legend(loc="lower right")
    a2.set_yscale("symlog", linthresh=100)
    a2.set(title="Fig 10b. Motor force (log scale): one huge spike when the target jumps", xlabel="time (s)", ylabel="force (N)"); a2.legend()
    save("fig10_kick.png"); R["kick"] = rows


def fig_sample_time():
    P = MassSpringDamper(); fig, ax = plt.subplots(figsize=(8, 4)); rows = []
    for dt in [0.001, 0.02, 0.05, 0.08, 0.1]:
        r = simulate(P, PID(**FINAL, dt=dt), 1.0, T=10, ctrl_dt=dt); m = metrics(r["t"], r["y"])
        unstable = (not np.isfinite(r["y"]).all()) or np.abs(r["y"]).max() > 3
        ax.plot(r["t"], np.clip(r["y"], -1, 3), label=f"update every {dt*1000:g} ms" + (" (UNSTABLE)" if unstable else f": overshoot {m['overshoot']:.1f} %"))
        rows.append(dict(dt_ms=dt * 1000, unstable=bool(unstable), **({} if unstable else rnd(m))))
    # find the stability edge
    lo, hi = 0.05, 0.1
    for _ in range(12):
        mid = (lo + hi) / 2; r = simulate(P, PID(**FINAL, dt=mid), 1.0, T=4, ctrl_dt=mid)
        if np.isfinite(r["y"]).all() and np.abs(r["y"][-2000:]).max() < 1.5: lo = mid
        else: hi = mid
    R["sample_time_limit_ms"] = round(lo * 1000, 1)
    ax.axhline(1, color="k", lw=1); ax.set_ylim(-1, 3); ax.set_xlim(0, 3)
    ax.set(title=f"Fig 11. Slower updates: fine at 20 ms, ringing at 50 ms, unstable beyond about {lo*1000:.0f} ms", xlabel="time (s)", ylabel="position (m, clipped)")
    ax.legend(loc="upper right"); save("fig11_sample_time.png"); R["sample_time"] = rows


# ---------------- Section 8: Ziegler-Nichols ----------------
def osc_ratio(plant, kp, T=40):
    r = simulate(plant, PID(kp, dt=0.01), 1.0, T=T, ctrl_dt=0.01); y = r["y"]; t = r["t"]
    pk = [i for i in range(1, len(y) - 1) if y[i] > y[i - 1] and y[i] >= y[i + 1]]
    if len(pk) < 4: return 0.0, np.nan, r
    yf = kp / (1 + kp); a = [y[i] - yf for i in pk]
    return a[-1] / a[-2] if a[-2] != 0 else 0.0, float(np.mean(np.diff(t[pk][-4:]))), r


def fig_zn():
    G1 = FirstOrderChain(n=1)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.6))
    for kp in [1, 10, 100]:
        r = simulate(G1, PID(kp, dt=0.01), 1.0, T=5, ctrl_dt=0.01); a1.plot(r["t"], r["y"], label=f"Kp = {kp}")
    a1.set(title="1/(s+1) under P control: never oscillates", xlabel="time (s)", ylabel="output"); a1.legend()
    G3 = FirstOrderChain(n=3); lo, hi = 4.0, 12.0
    for _ in range(25):
        mid = (lo + hi) / 2; ratio, _, _ = osc_ratio(G3, mid)
        if ratio < 1: lo = mid
        else: hi = mid
    Ku = (lo + hi) / 2; _, Pu, _ = osc_ratio(G3, Ku, T=60)
    for kp in [6, Ku, 10]:
        r = simulate(G3, PID(kp, dt=0.01), 1.0, T=30, ctrl_dt=0.01)
        a2.plot(r["t"], r["y"], label=f"Kp = {kp:.2f}" + (" (= Ku, steady swing)" if kp == Ku else ""))
    a2.set(title="1/(s+1)^3 under P control", xlabel="time (s)", ylabel="output", ylim=(-0.5, 3.2)); a2.legend(loc="upper left", fontsize=8, ncol=3)
    fig.suptitle("Fig 12. Step 1 of Ziegler-Nichols: raise Kp until the output swings steadily")
    save("fig12_zn_ultimate.png")
    zn = dict(kp=0.6 * Ku, ki=0.6 * Ku / (Pu / 2), kd=0.6 * Ku * Pu / 8)
    imp = dict(kp=0.6 * Ku * 0.6, ki=0.6 * Ku * 0.6 / (Pu * 1.0), kd=0.6 * Ku * 0.6 * Pu / 6)
    fig, ax = plt.subplots(figsize=(8, 4)); rows = []
    for lab, g in [("Ziegler-Nichols PID", zn), ("Improved by hand", imp)]:
        r = simulate(G3, PID(**g, dt=0.01), 1.0, T=25, ctrl_dt=0.01); m = metrics(r["t"], r["y"])
        ax.plot(r["t"], r["y"], label=f"{lab} (Kp={g['kp']:.2f}, Ki={g['ki']:.2f}, Kd={g['kd']:.2f}): overshoot {m['overshoot']:.1f} %, settle {m['settle']:.1f} s")
        rows.append(dict(case=lab, **{k: round(v, 3) for k, v in g.items()}, **rnd(m)))
    ax.axhline(1, color="k", lw=1); ax.set(title="Fig 13. Ziegler-Nichols result on 1/(s+1)^3, then improved", xlabel="time (s)", ylabel="output")
    ax.legend(loc="lower right", fontsize=8); save("fig13_zn_tuned.png")
    R["zn"] = dict(Ku=round(Ku, 3), Pu=round(Pu, 3), Ku_theory=8.0, Pu_theory=round(2 * math.pi / math.sqrt(3), 3), rows=rows)


# ---------------- Section 9: the final controller ----------------
def fig_final():
    P = MassSpringDamper(); sp = lambda t: 0.0 if t < 0.2 else (1.0 if t < 8.0 else 0.5)
    c = PID(**FINAL, u_min=-30, u_max=30, anti_windup=True, d_on_measurement=True, d_filter_tau=0.01)
    r = simulate(P, c, sp, T=12, noise=0.001)
    seg = (r["t"] >= 0.2) & (r["t"] < 8.0); t = r["t"][seg] - 0.2; m = metrics(t, r["y"][seg])
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
    a1.plot(r["t"], r["y"], label="position"); a1.plot(r["t"], r["r"], "k:", label="target")
    a1.set(title="Fig 14a. Final PID (all fixes) with 30 N limit and +/-1 mm noise", ylabel="position (m)"); a1.legend(loc="upper right")
    a2.plot(r["t"], r["u"], lw=0.8, label="motor force"); a2.axhline(30, color="r", ls="--", lw=0.8, label="+/-30 N limit"); a2.axhline(-30, color="r", ls="--", lw=0.8)
    a2.set(title="Fig 14b. Motor force stays inside the limit and is smooth", xlabel="time (s)", ylabel="force (N)"); a2.legend(loc="upper right")
    save("fig14_final.png"); R["final_step"] = rnd(m)
    # what-if runs (practice for the interview)
    fig, ax = plt.subplots(figsize=(8, 4)); rows = []
    for mass in [0.5, 1, 2, 4]:
        c = PID(**FINAL, u_min=-30, u_max=30, anti_windup=True, d_on_measurement=True, d_filter_tau=0.01)
        r = simulate(MassSpringDamper(m=mass), c, 1.0, T=12, noise=0.001); mm = metrics(r["t"], r["y"])
        ax.plot(r["t"], r["y"], label=f"mass {mass} kg: rise {mm['rise']:.2f} s, overshoot {mm['overshoot']:.1f} %")
        rows.append(dict(mass=mass, **rnd(mm)))
    ax.axhline(1, color="k", lw=1); ax.set_xlim(0, 4); ax.set(title="Fig 15. Same controller, different mass (heavier = slower under the 30 N limit)", xlabel="time (s)", ylabel="position (m)")
    ax.legend(loc="lower right"); save("fig15_mass_whatif.png"); R["mass_whatif"] = rows


if __name__ == "__main__":
    for f in [fig_loop, fig_open_loop, fig_metrics, fig_parts, fig_compare, fig_windup, fig_noise,
              fig_kick, fig_sample_time, fig_zn, fig_final]:
        print("running", f.__name__); f()
    with open(os.path.join(RES, "results.json"), "w") as fh:
        json.dump(R, fh, indent=1, default=float)
    print("done: figures/ and results/results.json")
