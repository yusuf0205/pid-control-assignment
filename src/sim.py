"""Closed-loop simulation and the four performance measures."""
import numpy as np
from plants import rk4_step

DT_SIM = 0.0005          # plant integration step (s): the "real world" runs finer than the controller


def simulate(plant, controller, setpoint, T=5.0, ctrl_dt=0.001, noise=0.0, u_limit=None,
             open_loop_force=None, seed=0):
    """Simulate T seconds.

    setpoint        : float or function t -> setpoint
    controller      : PID object (its dt must equal ctrl_dt), or None for open loop
    noise           : sensor noise amplitude (uniform in [-noise, +noise])
    open_loop_force : function t -> force, used when controller is None
    The controller output is held constant between updates (zero-order hold).
    Returns dict of arrays: t, y (true output), y_meas, u, r.
    """
    rng = np.random.default_rng(seed)
    sp = setpoint if callable(setpoint) else (lambda t, v=setpoint: v)
    plant.reset()
    if controller is not None:
        controller.reset()
    n = int(round(T / DT_SIM))
    every = max(1, int(round(ctrl_dt / DT_SIM)))
    t = np.arange(n + 1) * DT_SIM
    y = np.zeros(n + 1); ym = np.zeros(n + 1); u_log = np.zeros(n + 1); r = np.zeros(n + 1)
    u = 0.0
    for i in range(n + 1):
        ti = t[i]
        y[i] = plant.output
        r[i] = sp(ti)
        if i % every == 0:
            meas = y[i] + (rng.uniform(-noise, noise) if noise > 0 else 0.0)
            ym[i] = meas
            if controller is not None:
                u = controller.update(r[i], meas)
            else:
                u = open_loop_force(ti)
            if u_limit is not None:
                u = float(np.clip(u, -u_limit, u_limit))
        else:
            ym[i] = ym[i - 1]
        u_log[i] = u
        if i < n:
            rk4_step(plant, u, DT_SIM)
    return dict(t=t, y=y, y_meas=ym, u=u_log, r=r)


def metrics(t, y, setpoint=1.0, band=0.02):
    """Rise time (10->90 % of final value), overshoot (% of final value),
    settling time (stays within +/-2 % of final value), steady-state error (setpoint - final)."""
    tail = max(1, len(y) // 20)
    y_final = float(np.mean(y[-tail:]))
    if abs(y_final) < 1e-9:
        return dict(rise=np.nan, overshoot=np.nan, settle=np.nan, sse=setpoint, y_final=y_final)
    try:
        t10 = t[np.argmax(y >= 0.1 * y_final)]
        t90 = t[np.argmax(y >= 0.9 * y_final)]
        rise = t90 - t10
    except Exception:
        rise = np.nan
    overshoot = max(0.0, (np.max(y) - y_final) / abs(y_final) * 100.0)
    outside = np.where(np.abs(y - y_final) > band * abs(y_final))[0]
    settle = t[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(t) else (t[0] if not len(outside) else np.nan)
    return dict(rise=rise, overshoot=overshoot, settle=settle, sse=setpoint - y_final, y_final=y_final)
