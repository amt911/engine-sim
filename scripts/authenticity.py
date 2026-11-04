#!/usr/bin/env python3
"""How much of an engine sound repeats from one engine cycle to the next.

A synthetic engine is too periodic: each cycle a copy of the last. A real one carries turbulent
noise and cycle-to-cycle variation riding on its pulses. This cuts a steady-rpm stretch into
blocks of engine cycles (two crank turns each), averages each block's cycles (the synchronous
average: the deterministic part) and measures, per band, the share of energy left over (the
stochastic part), plus the correlation between consecutive cycles and the harmonic-to-noise ratio
of the engine orders. Blocks are short (8 cycles) so that real rpm wander stays inside one and is
not counted as noise.

Usage:
  scripts/authenticity.py FILE.wav[@T0[:DUR]][~RPM] ...   T0 start (s), DUR length (default 2 s),
                                                           RPM a hint (else found from the signal)
  scripts/authenticity.py --self-test
Reads 16/24-bit PCM and 32-bit float WAV (stereo is averaged to mono).
"""
import re
import sys
import numpy as np

BANDS = [(63, 250), (250, 1000), (1000, 4000), (4000, 12000)]
BLOCK = 8  # cycles per block
UP = 8  # band-limited oversampling before the (linear) cycle-grid interpolation
PER_CYCLE = 4096  # points per cycle on the cycle grid


def load(path):
    b = open(path, "rb").read()
    i, fs, data, fmt, bits, ch = 12, None, None, None, None, 1
    while i < len(b) - 8:
        cid, n = b[i:i + 4], int.from_bytes(b[i + 4:i + 8], "little")
        if cid == b"fmt ":
            fmt = int.from_bytes(b[i + 8:i + 10], "little")
            ch = int.from_bytes(b[i + 10:i + 12], "little")
            fs = int.from_bytes(b[i + 12:i + 16], "little")
            bits = int.from_bytes(b[i + 22:i + 24], "little")
        if cid == b"data":
            raw = b[i + 8:i + 8 + n]
            if fmt == 3 or (fmt == 0xFFFE and bits == 32):
                data = np.frombuffer(raw[: len(raw) // 4 * 4], dtype="<f4").astype(float)
            elif bits == 24:
                a = np.frombuffer(raw[: len(raw) // 3 * 3], dtype=np.uint8).reshape(-1, 3)
                v = (a[:, 0].astype(np.int32) | (a[:, 1].astype(np.int32) << 8)
                     | (a[:, 2].astype(np.int32) << 16))
                v = np.where(v & 0x800000, v - 0x1000000, v)
                data = v.astype(float) / 8388608.0
            else:
                data = np.frombuffer(raw[: len(raw) // 2 * 2], dtype="<i2").astype(float) / 32768.0
        i += 8 + n + (n & 1)
    if ch > 1:
        data = data[: len(data) // ch * ch].reshape(-1, ch).mean(axis=1)
    return fs, data


def band_energy(x, fs, lo, hi):
    p = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), 1 / fs)
    return p[(f >= lo) & (f < hi)].sum()


def upsample(x, k=UP):
    """`x` at k× its rate, band-limited (zero-padded spectrum): linear interpolation on this
    grid errs by (π·f/(k·fs))²/2, under −45 dB at 12 kHz from 48 kHz."""
    n = len(x)
    X = np.fft.rfft(x)
    Y = np.zeros(n * k // 2 + 1, dtype=complex)
    Y[: len(X)] = X
    return np.fft.irfft(Y, n * k) * k


def lowpass(x, fs, fc):
    """`x` with everything above `fc` removed (brick-wall, in the spectrum)."""
    X = np.fft.rfft(x)
    X[np.fft.rfftfreq(len(x), 1 / fs) > fc] = 0
    return np.fft.irfft(X, len(x))


def track(x, fs, hz, limit=None, chirp_span=0.0):
    """Frequency and ramp of the cycles in `x`, coarse to fine: the residual's basin around the
    right frequency is ~1/(f_max·duration) wide (sharp pulses misaligned by more than their
    width all look alike), so the search runs first on a 200 Hz low-pass, then 1.5 kHz, then the
    full band, each narrowing the span."""
    a = 0.0
    for fc, span, cs in ((200, 0.02, chirp_span), (1500, 0.003, chirp_span / 5),
                         (None, 0.0005, chirp_span / 25)):
        y = x if fc is None else lowpass(x, fs, fc)
        mid0 = hz + a * (limit or len(x) / fs * hz) / hz / 2
        f0, a = refine(y, fs, mid0, span, limit=limit, chirp_span=cs, a0=a)
        hz = f0
    return hz, a


def find_cycle_hz(x, fs, hint=None):
    """Coarse engine-cycle frequency (rpm/120): the lag whose normalised autocorrelation is
    highest, near the hint or over 4–75 Hz."""
    lo, hi = (hint / 120 * 0.85, hint / 120 * 1.15) if hint else (4.0, 75.0)
    best, best_r = None, -2.0
    for hz in np.linspace(lo, hi, 400):
        n = int(round(fs / hz))
        if 3 * n > len(x):
            continue
        a, b = x[:-n], x[n:]
        r = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-30)
        if r > best_r:
            best, best_r = hz, r
    return best


def cycles(x, fs, cyc_hz, limit=None, chirp=0.0):
    """`x` resampled on the engine-cycle grid: one row per whole cycle, PER_CYCLE points. The
    cycle phase is φ(t) = f·t + ½·a·t² (cycles; a = `chirp`, Hz/s), so a steady rpm ramp inside
    the stretch keeps the cycles aligned."""
    T = len(x) / fs
    m = int(cyc_hz * T + 0.5 * chirp * T * T - 1e-9)
    if limit:
        m = min(m, limit)
    ph = np.arange(m * PER_CYCLE) / PER_CYCLE
    if abs(chirp) < 1e-12:
        t = ph / cyc_hz
    else:
        t = 2 * ph / (cyc_hz + np.sqrt(cyc_hz * cyc_hz + 2 * chirp * ph))
    t = t * fs
    if m < 1 or t[-1] > len(x) - 1:
        return np.zeros((0, PER_CYCLE)), PER_CYCLE * cyc_hz
    return np.interp(t, np.arange(len(x)), x).reshape(m, PER_CYCLE), PER_CYCLE * cyc_hz


def residual_share(c):
    """Residual energy over total for cycles `c` (rows) around their mean, corrected for the mean
    containing each cycle's own noise (noise leaves (N−1)/N of itself in the residual)."""
    n = len(c)
    r = c - c.mean(axis=0)
    return (r ** 2).sum() * n / (n - 1) / ((c ** 2).sum() + 1e-30)


def refine(x, fs, hz, span, steps=21, limit=None, chirp_span=0.0, a0=0.0):
    """The cycle frequency (and, with `chirp_span`, its rate of change, as a fraction of `hz`
    per second) near `hz` that leaves the least residual, narrowed four times by 5: the phase
    slip at order k across a block grows as k·BLOCK·Δf/f, so at 10 kHz the frequency must be
    right to ~1e-6."""
    # searched as (mid-stretch frequency, ramp): unlike (start frequency, ramp) the two barely
    # trade off against each other, so a coordinate search converges
    n_cyc = limit or len(x) / fs * hz
    mid = lambda fm, aa: fm - aa * n_cyc / fm / 2  # start frequency for that mid and ramp
    fm, a = hz, a0
    cs = chirp_span * hz
    for _ in range(6):
        best, best_e = (fm, a), np.inf
        grid = [(f, a) for f in fm * (1 + np.linspace(-span, span, steps))]
        if cs:
            grid += [(fm, aa) for aa in a + np.linspace(-cs, cs, steps)]
        for f, aa in grid:
            c, _ = cycles(x, fs, mid(f, aa), limit, aa)
            if len(c) < 2:
                continue
            e = residual_share(c)
            if e < best_e:
                best, best_e = (f, aa), e
        (fm, a), span, cs = best, span / 4, cs / 4
    return mid(fm, a), a


def measure(x, fs, rpm_hint=None):
    hz0 = find_cycle_hz(x[::4], fs / 4, rpm_hint)  # coarse, on a decimated copy
    hz0, _ = refine(lowpass(x, fs, 200), fs, hz0, span=0.02)
    blk = int(BLOCK * fs / hz0)
    spare = int(1.5 * fs / hz0)
    xu, fsu = upsample(x), fs * UP
    e_all, e_res = np.zeros(len(BANDS)), np.zeros(len(BANDS))
    corrs, hzs, harm, rest_lines = [], [], 0.0, 0.0
    for s0 in range(0, len(x) - blk - spare + 1, blk):
        seg = xu[s0 * UP:(s0 + blk + spare) * UP]
        # follow the rpm block to block: a coarse look near the last block's, then frequency
        # and ramp together (two passes: each settles better once the other is close)
        hz = find_cycle_hz(x[s0:s0 + blk + spare][::4], fs / 4, hz0 * 120)
        hz, a = track(seg, fsu, hz, limit=BLOCK, chirp_span=0.05)
        hz0 = hz + a * BLOCK / hz / 2  # the block's mid frequency, next block's start guess
        hzs.append(hz0)
        c, fs_c = cycles(seg, fsu, hz, BLOCK, a)
        n = len(c)
        resid = (c - c.mean(axis=0)) * np.sqrt(n / (n - 1))
        whole, rest = c.reshape(-1), resid.reshape(-1)
        for i, (lo, hi) in enumerate(BANDS):
            hi = min(hi, 0.45 * fs)
            e_all[i] += band_energy(whole, fs_c, lo, hi)
            e_res[i] += band_energy(rest, fs_c, lo, hi)
        corrs += [np.dot(c[k], c[k + 1]) / (np.linalg.norm(c[k]) * np.linalg.norm(c[k + 1]) + 1e-30)
                  for k in range(n - 1)]
        # harmonic-to-noise of the engine orders, 20 Hz–4 kHz: over a block of BLOCK whole
        # cycles on the cycle grid, the orders fall exactly on every BLOCK-th FFT bin
        p = np.abs(np.fft.rfft(whole)) ** 2
        k = np.arange(len(p))
        f = k * fs_c / len(whole)
        sel = (f > 20) & (f < 4000)
        on = sel & (k % BLOCK == 0)
        harm += p[on].sum()
        rest_lines += p[sel & ~on].sum()
    shares = [10 * np.log10(r / (a + 1e-30) + 1e-30) for r, a in zip(e_res, e_all)]
    return dict(rpm=float(np.mean(hzs) * 120), rpm_sd=float(np.std(hzs) * 120),
                cycles=len(hzs) * BLOCK, shares=[float(v) for v in shares],
                corr=float(np.mean(corrs)), hnr=float(10 * np.log10(harm / (rest_lines + 1e-30))))


def total_share(m):
    return 10 * np.log10(sum(10 ** (v / 10) for v in m["shares"][:2]) / 2)


def measure_guess(x, fs, f0):
    """For a recording labelled only with a pitch f0 whose relation to the cycle is unknown
    (a tracker may lock on the crank or the cycle rate): try f0/2, f0 and 2·f0 as the cycle and
    keep the fastest whose residual below 1 kHz is within 1 dB of the best. The true cycle
    always repeats; twice it repeats as well but adds nothing."""
    tries = []
    for k in (2.0, 1.0, 0.5):
        cyc = f0 / k
        if not 3 <= cyc <= 80 or len(x) < 3 * BLOCK * fs / cyc:
            continue
        tries.append(measure(x, fs, cyc * 120))
    best = min(total_share(m) for m in tries)
    return max((m for m in tries if total_share(m) <= best + 1.0), key=lambda m: m["rpm"])


def self_test():
    rng = np.random.default_rng(1)
    fs, rpm = 48000, 3000.0
    cyc = rpm / 120
    t = np.arange(int(3 * fs)) / fs
    # four sharp pulses a cycle (a four-cylinder; σ ≈ 30 µs: content to ~10 kHz)
    pulses = np.exp(-((((t * cyc * 4) % 1.0) - 0.1) / 0.004) ** 2)
    det = pulses - pulses.mean()
    m = measure(det, fs, rpm)
    assert abs(m["rpm"] - rpm) < 1, m
    assert all(s < -30 for s in m["shares"]), m  # a pure copy: nothing stochastic
    assert m["corr"] > 0.999, m
    # white noise in the 4–12 kHz band at the same energy as the pulses there: the residual is
    # half that band (−3 dB) and the lower bands stay clean
    noise = rng.standard_normal(len(t))
    F = np.fft.rfft(noise)
    f = np.fft.rfftfreq(len(t), 1 / fs)
    F[(f < 4000) | (f >= 12000)] = 0
    noise = np.fft.irfft(F, len(t))
    D = np.fft.rfft(det)
    D[(f < 4000) | (f >= 12000)] = 0
    noise *= np.sqrt(np.mean(np.fft.irfft(D, len(t)) ** 2) / np.mean(noise ** 2))
    m2 = measure(det + noise, fs, rpm)
    assert abs(m2["shares"][3] - (-3.0)) < 0.4, m2
    assert all(s < -25 for s in m2["shares"][:3]), m2
    # rpm drifting 2 %/s (a slow real-world wander) must not read as noise
    phase = 2 * np.pi * cyc * 4 * (t + 0.01 * t ** 2)
    drift = np.exp(-(((phase / (2 * np.pi)) % 1.0 - 0.1) / 0.004) ** 2)
    m3 = measure(drift - drift.mean(), fs, rpm)
    assert all(s < -20 for s in m3["shares"][:3]), m3
    print("self-test ok")
    for name, r in (("periodic", m), ("+noise 4-12k at -3 dB", m2), ("rpm drift 2%/s", m3)):
        print(f"  {name:22s} shares {np.round(r['shares'], 1).tolist()} corr {r['corr']:.4f} "
              f"HNR {r['hnr']:.1f}")


def main(args):
    if args == ["--self-test"]:
        self_test()
        return
    hdr = "  ".join(f"{lo}-{hi}" for lo, hi in BANDS)
    print(f"{'file':52s} {'rpm':>10s} cyc  stochastic dB [{hdr}]   corr    HNR")
    for a in args:
        rpm = None
        if "~" in a:
            a, r = a.rsplit("~", 1)
            rpm = float(r)
        path, t0, dur = a, 0.0, 2.0
        if "@" in a:
            path, spec = a.rsplit("@", 1)
            t0, dur = map(float, spec.split(":")) if ":" in spec else (float(spec), dur)
        fs, x = load(path)
        seg = x[int(t0 * fs): int((t0 + dur) * fs)]
        f0 = re.search(r"f0_(\d+)Hz", path)
        m = measure(seg, fs, rpm) if rpm or not f0 else measure_guess(seg, fs, float(f0.group(1)))
        name = path.split("/")[-1][-52:]
        sh = " ".join(f"{v:6.1f}" for v in m["shares"])
        print(f"{name:52s} {m['rpm']:6.0f}±{m['rpm_sd']:<3.0f} {m['cycles']:3d}  {sh}   "
              f"{m['corr']:.3f} {m['hnr']:6.1f}")


if __name__ == "__main__":
    main(sys.argv[1:])
