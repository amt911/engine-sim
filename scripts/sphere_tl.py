#!/usr/bin/env python3
"""Transmission loss of a rigid spherical chamber with two circular ports on its wall, by the
modal impedance matrix: the sphere's Neumann Green's function (modes j_l(k r) P_l(cos θ) with
j_l'(k a) = 0, norm a³/2 · j_l(ka)² (1 - l(l+1)/(ka)²) · 4π/(2l+1); Morse & Ingard, Theoretical
Acoustics §9.4), averaged over each port's cap, then the two-port TL with an anechoic outlet as in
the impedance-matrix method (Selamet & Ji, J. Sound Vib. 223(2), 1999). A loss factor η on k
stands for the wall and boundary-layer losses the rigid model lacks.

Why: the LFA's final chamber (JP 2008-169751 A) is a sphere; the patent gives its loss only to
1 kHz (Fig. 6). Fitted to those points (a, port radius, port angle), the model predicts the loss
above. Checks: at 100 Hz the a = 0.15 m sphere with 42.5 mm ports gives 4.6 dB, the lumped volume
gives 20·log10|1 + jωV/(2cS)| = 4.9 dB by hand.

Usage: uv run --with scipy --with numpy scripts/sphere_tl.py A_M PORT_R_M ANGLE_DEG [ETA]
Prints the TL at the patent's frequencies, then power-averaged half-octaves 1-8 kHz.
"""
import sys
import numpy as np
from scipy.special import spherical_jn, eval_legendre
from scipy.optimize import brentq

C0, RHO = 540.0, 0.45  # tail gas at ≈ 720 K


def roots_djl(l, xmax):
    f = lambda x: spherical_jn(l, x, derivative=True)
    xs = np.linspace(1e-6, xmax, int(xmax * 20) + 50)
    v = f(xs)
    out = [brentq(f, xs[i], xs[i + 1]) for i in range(len(xs) - 1) if v[i] * v[i + 1] < 0]
    out = [r for r in out if r > 1e-3]
    return ([0.0] if l == 0 else []) + out


def cap_integral(l, c):
    """∫_c^1 P_l(x) dx."""
    if l == 0:
        return 1 - c
    return (eval_legendre(l - 1, c) - eval_legendre(l + 1, c)) / (2 * l + 1)


def modes(a, r1, theta2, xmax=120, lmax=80):
    """(k, port-1 average, port-2 average, norm) per mode; port 1 at θ = 0, port 2 at θ2."""
    c = np.sqrt(1 - (r1 / a) ** 2)
    S = 2 * np.pi * a * a * (1 - c)
    out = []
    for l in range(lmax + 1):
        f1 = 2 * np.pi * a * a * cap_integral(l, c)
        f2 = f1 * eval_legendre(l, np.cos(theta2))  # addition theorem, axisymmetric cap
        for x in roots_djl(l, xmax):
            if x == 0:
                j, n = 1.0, 4 / 3 * np.pi * a ** 3
            else:
                j = spherical_jn(l, x)
                n = a ** 3 / 2 * j * j * (1 - l * (l + 1) / x ** 2) * 4 * np.pi / (2 * l + 1)
            if np.isfinite(j * f1 * f2 * n) and n > 0:
                out.append((x / a, j * f1 / S, j * f2 / S, n))
    return out, S


def tl(f, m, S, eta=0.01):
    w = 2 * np.pi * f
    k = w / C0 * (1 - 0.5j * eta)
    z = np.zeros((2, 2), complex)
    for km, a1, a2, n in m:
        d = n * (km ** 2 - k ** 2)
        z[0, 0] += a1 * a1 / d
        z[0, 1] += a1 * a2 / d
        z[1, 1] += a2 * a2 / d
    z *= 1j * w * RHO
    y0 = RHO * C0 / S
    p2 = z[0, 1] / (1 + z[1, 1] / y0)  # per unit inlet volume velocity
    p1 = z[0, 0] - z[0, 1] * p2 / y0
    return 20 * np.log10(abs((p1 + y0) / 2 / p2))


def main(argv):
    a, r1, ang = float(argv[0]), float(argv[1]), np.radians(float(argv[2]))
    eta = float(argv[3]) if len(argv) > 3 else 0.01
    m, S = modes(a, r1, ang)
    for f in [50, 100, 150, 200, 300, 500, 700, 1000]:
        print(f"{f:6} Hz {tl(f, m, S, eta):6.1f} dB")
    for fc in [1414, 2000, 2828, 4000, 5657, 8000]:
        fs = np.geomspace(fc / 2 ** 0.25, fc * 2 ** 0.25, 160)
        t = np.array([tl(x, m, S, eta) for x in fs])
        print(f"{fc:6} Hz {-10 * np.log10(np.mean(10 ** (-t / 10))):6.1f} dB (half-octave power mean)")


if __name__ == "__main__":
    main(sys.argv[1:])
