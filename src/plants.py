"""Plants (the machines being controlled), integrated with 4th-order Runge-Kutta."""
import numpy as np


class MassSpringDamper:
    """m*x'' + c*x' + k*x = F   (default: m = 1 kg, c = 10 N*s/m, k = 20 N/m)."""
    def __init__(self, m=1.0, c=10.0, k=20.0):
        self.m, self.c, self.k = m, c, k
        self.state = np.zeros(2)          # [position x (m), velocity v (m/s)]

    def reset(self):
        self.state = np.zeros(2)

    def deriv(self, s, F):
        x, v = s
        return np.array([v, (F - self.c * v - self.k * x) / self.m])

    @property
    def output(self):
        return self.state[0]


class FirstOrderChain:
    """n identical first-order lags in series: G(s) = 1 / (s + 1)^n   (time constant 1 s each)."""
    def __init__(self, n=3, tau=1.0):
        self.n, self.tau = n, tau
        self.state = np.zeros(n)

    def reset(self):
        self.state = np.zeros(self.n)

    def deriv(self, s, u):
        d = np.empty(self.n)
        d[0] = (u - s[0]) / self.tau
        for i in range(1, self.n):
            d[i] = (s[i - 1] - s[i]) / self.tau
        return d

    @property
    def output(self):
        return self.state[-1]


def rk4_step(plant, u, h):
    s = plant.state
    k1 = plant.deriv(s, u)
    k2 = plant.deriv(s + 0.5 * h * k1, u)
    k3 = plant.deriv(s + 0.5 * h * k2, u)
    k4 = plant.deriv(s + h * k3, u)
    plant.state = s + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
