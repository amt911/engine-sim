#!/usr/bin/env python3
"""Fit the narrow-tube (Zwikker & Kosten) functions as passive Foster networks for the time domain.

A circular tube of radius a carries plane waves with an effective density and compressibility
(Zwikker & Kosten 1949; Stinson, JASA 89, 1991):
    rho_eff / rho = 1 / (1 - F(s*sqrt(-i)))                       viscous, s = a*sqrt(omega/nu)
    C_eff / C_0   = 1 + (gamma - 1) * F(s*sqrt(Pr)*sqrt(-i))       thermal
    F(z) = 2*J1(z) / (z*J0(z))
In the dimensionless frequency x = omega*a^2/nu = s^2:
    zeta(x) = i*x*(rho_eff/rho - 1)      excess series impedance, in units of mu/a^2 (per area)
    psi(x)  = i*x*F(s*sqrt(Pr)*sqrt(-i)) excess shunt admittance, in units of (gamma-1)*nu/(gamma*p*a^2)
Each is fitted with fixed, log-spaced time constants tau_k (linear least squares):
    zeta(x) ~ r0 + sum_k r_k * (i x tau_k)/(1 + i x tau_k)    parallel-RL branches (+ a resistor)
    psi(x)  ~      sum_k g_k * (i x tau_k)/(1 + i x tau_k)    series-RC branches
Every coefficient must come out positive (a passive network). Prints Rust constants.

Usage: python3 -I scripts/fit_narrow_tube.py
"""
import numpy as np

PR = 0.71  # Prandtl number of air and exhaust gas (nearly constant with temperature)


def j1_over_j0(z):
    """J1(z)/J0(z) by backward recurrence of J_n/J_{n-1} = 1/(2n/z - J_{n+1}/J_n) (stable for any z)."""
    n_top = int(abs(z)) + 60
    r = 0.0 + 0.0j
    for n in range(n_top, 0, -1):
        r = 1.0 / (2.0 * n / z - r)
    return r


def f_zk(z):
    """F(z) = 2 J1(z) / (z J0(z)); its series 1 - z^2/8 near zero."""
    if abs(z) < 1e-3:
        return 1.0 - z * z / 8.0
    return 2.0 / z * j1_over_j0(z)


ROOT_MI = np.exp(-1j * np.pi / 4)  # sqrt(-i)


def zeta(s):
    x = s * s
    f = f_zk(s * ROOT_MI)
    return 1j * x * (1.0 / (1.0 - f) - 1.0)


def psi(s):
    x = s * s
    return 1j * x * f_zk(s * np.sqrt(PR) * ROOT_MI)


def nnls(a, b, iters=500):
    """Lawson & Hanson's non-negative least squares (Solving Least Squares Problems, 1974, ch. 23)."""
    n = a.shape[1]
    x = np.zeros(n)
    passive = np.zeros(n, dtype=bool)
    w = a.T @ (b - a @ x)
    for _ in range(iters):
        if passive.all() or w[~passive].max() <= 1e-12:
            break
        j = np.argmax(np.where(passive, -np.inf, w))
        passive[j] = True
        while True:
            z = np.zeros(n)
            z[passive], *_ = np.linalg.lstsq(a[:, passive], b, rcond=None)
            if (z[passive] > 0).all():
                x = z
                break
            neg = passive & (z <= 0)
            alpha = np.min(x[neg] / (x[neg] - z[neg]))
            x = x + alpha * (z - x)
            passive &= x > 1e-15
        w = a.T @ (b - a @ x)
    return x


def fit(fun, with_r0, s_lo=0.3, s_hi=60.0, n_branch=8, margin=10.0):
    s = np.geomspace(s_lo, s_hi, 400)
    x = s * s
    target = np.array([fun(v) for v in s])
    taus = np.geomspace(1.0 / (margin * s_hi * s_hi), margin / (s_lo * s_lo), n_branch)
    cols = [(1j * x * t) / (1.0 + 1j * x * t) for t in taus]
    if with_r0:
        cols = [np.ones_like(x, dtype=complex)] + cols
    a = np.array(cols).T
    w = 1.0 / np.abs(target)  # relative error
    aw = a * w[:, None]
    bw = target * w
    m = np.vstack([aw.real, aw.imag])
    b = np.concatenate([bw.real, bw.imag])
    coef = nnls(m, b)
    approx = a @ coef
    err = np.max(np.abs(approx - target) / np.abs(target))
    return taus, coef, err


def main():
    # checks on the exact functions: Poiseuille 8 and the 4/3 mass at low x; psi -> i x at low x
    z0 = zeta(0.05)
    assert abs(z0.real - 8.0) < 1e-3 and abs(z0.imag / 0.0025 - 1.0 / 3.0) < 1e-2, z0
    for n in (8, 10, 12, 14, 16):
        tv, cv, ev = fit(zeta, True, n_branch=n)
        tt, ct, et = fit(psi, False, n_branch=n)
        ok = (cv >= 0).all() and (ct >= 0).all()
        print(f"# {n} branches: viscous max rel err {ev:.3%}, thermal {et:.3%}, passive {ok}")
    n = 10
    tv, cv, ev = fit(zeta, True, n_branch=n)
    tt, ct, et = fit(psi, False, n_branch=n)
    assert (cv >= 0).all() and (ct >= 0).all(), (cv, ct)
    print(f"// fit_narrow_tube.py, {n} branches, 0.3 <= s <= 60: max relative error "
          f"viscous {ev:.2%}, thermal {et:.2%}")
    print(f"const VISCOUS_R0: f64 = {float(cv[0])!r};")
    print("const VISCOUS: [(f64, f64); %d] = [" % n)
    for t, r in zip(tv, cv[1:]):
        print(f"    ({float(t)!r}, {float(r)!r}),")
    print("];")
    print("const THERMAL: [(f64, f64); %d] = [" % n)
    for t, g in zip(tt, ct):
        print(f"    ({float(t)!r}, {float(g)!r}),")
    print("];")


if __name__ == "__main__":
    main()
