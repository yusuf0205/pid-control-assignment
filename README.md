# PID Control from Scratch

Modern Monk Labs, Physical AI Research Intern assignment. Mohammed Yusuf, NIT Puducherry.

A beginner-friendly explanation of PID control, backed by formulas and by Python simulations written from scratch (NumPy and Matplotlib only, no control library). The full report is `PID_Control_Report_Mohammed_Yusuf.pdf`.

## Quick start

```bash
git clone https://github.com/yusuf0205/pid-control-assignment.git
cd pid-control-assignment
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python src/make_figures.py
```

This recreates all 15 graphs in `figures/` and every number quoted in the report in `results/results.json`. It takes about one minute. The results are deterministic (sensor noise uses a fixed random seed of 0), so anyone gets identical graphs.

## Repository layout

| Path | What it is |
|---|---|
| `src/pid.py` | The reusable `PID` class: P, I, D plus output limits, anti-windup, derivative-on-measurement and a low-pass D filter |
| `src/plants.py` | The machines being controlled: the mass-spring-damper (m = 1 kg, c = 10 N s/m, k = 20 N/m) and the 1/(s+1)^n lag chain, with an RK4 integrator |
| `src/sim.py` | The closed-loop simulator (plant at 0.5 ms, controller at a chosen rate with zero-order hold, optional sensor noise) and the four performance measures |
| `src/make_figures.py` | One function per report section; produces every graph and `results/results.json` |
| `figures/` | Generated graphs (fig01 to fig15) |
| `results/results.json` | Every number used in the report |
| `docs/report.md` | Source of the PDF report |
| `PID_Control_Report_Mohammed_Yusuf.pdf` | The submitted document (11 pages) |

## Graph to code map

| Figure | Report section | Function in `make_figures.py` |
|---|---|---|
| fig01 loop diagram | 1 | `fig_loop` |
| fig02 open loop, constant push | 3 | `fig_open_loop` |
| fig03 four measures marked | 4 | `fig_metrics` |
| fig04 P only, fig05 add I, fig06 add D | 5 | `fig_parts` |
| fig07 P vs PI vs PD vs PID | 6 | `fig_compare` |
| fig08 windup (30 N limit) | 7a | `fig_windup` |
| fig09 sensor noise (+/-1 mm) | 7b | `fig_noise` |
| fig10 derivative kick | 7c | `fig_kick` |
| fig11 slow update rate | 7d | `fig_sample_time` |
| fig12, fig13 Ziegler-Nichols | 8 | `fig_zn` |
| fig14 final controller, fig15 mass what-if | 9 | `fig_final` |

## Using the PID class on your own

```python
import sys; sys.path.insert(0, "src")
from pid import PID
from plants import MassSpringDamper
from sim import simulate, metrics

c = PID(kp=300, ki=200, kd=20, dt=0.001, u_min=-30, u_max=30,
        anti_windup=True, d_on_measurement=True, d_filter_tau=0.01)
r = simulate(MassSpringDamper(m=2.0), c, setpoint=1.0, T=8, noise=0.001)
print(metrics(r["t"], r["y"]))
```

Change any gain, the mass, the noise, or the force limit and rerun to see the effect.

## Approach

1. Model the plant (Newton's law), then predict by hand: final position under P only is Kp/(20+Kp), damping ratio is (10+Kd)/(2*sqrt(20+Kp)).
2. Simulate and compare with the prediction (they agree to 3 decimals for the final position and within 0.6 % for overshoot).
3. Add real-world limits one at a time (30 N limit, +/-1 mm noise, a step target, slow updates), show each problem, then fix it.
4. Tune: by hand on the block, and with Ziegler-Nichols on 1/(s+1)^3.

## Results in brief

| Controller (Kp, Ki, Kd) | Rise | Overshoot | Settling | Final error |
|---|---|---|---|---|
| P (300, 0, 0) | 0.072 s | 40.7 % | 0.78 s | 0.063 m |
| PI (300, 200, 0) | 0.075 s | 36.9 % | 1.22 s | 0 |
| PD (300, 0, 20) | 0.146 s | 0.7 % | 0.23 s | 0.063 m |
| PID (300, 200, 20) | 0.153 s | 0.5 % | 0.24 s | 0 |
| Final PID, 30 N limit, noise, all fixes | 0.41 s | 0.01 % | 2.01 s | 0.0005 m |

Fixes: anti-windup cut overshoot from 11.2 % to 0.0 %; the D filter cut force noise from 17.4 N to 1.3 N (standard deviation); derivative-on-measurement cut the step kick from 20,300 N to 300 N. With these gains the loop goes unstable when the controller updates slower than about every 73 ms.

## What worked, what did not, what next

* **Worked:** theory and simulation agree; each fix removes its problem cleanly.
* **Did not work perfectly:** with the 30 N limit the block reaches 0.95 m fast, then creeps to 1 m (settling 2.07 s) while the integral rebuilds. Ziegler-Nichols on the brief's second system, 1/(s+1)^3, gives 52% overshoot with the classic table, so the gains were refined by hand to about 10%.
* **Next:** back-calculation anti-windup, a 20 N spring feed-forward term, a setpoint ramp that respects the force limit, and optimiser-based tuning on the realistic (limited, noisy) system.

## AI use

An AI assistant helped structure the code and draft and proofread the text. All numbers come from running this code and were checked against hand calculations.
