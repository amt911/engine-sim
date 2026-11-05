#!/usr/bin/env python3
"""How a sound's treble rides the firing rhythm, and its band balance.

The measure that separated every ear verdict on "synthetic" (docs/agents/debugging.md, *Sound
realism*): per band, the envelope's strongest line between 12 and 400 Hz over the envelope's
mean (%, the median of 2-s windows), and the band levels 300–1k / 1–3k / 3–8k Hz re 50–300 Hz
(dB). Real steady recordings: ≈ 12 % median in 0.8–5 kHz (209 segments, 2026-11-05).

usage: modulation.py WAV...            one line per file
       modulation.py --real DIR        the steady segments (*estable.wav) of DIR: median, p25, p75
       modulation.py --self-test
"""
import glob, os, sys
import numpy as np

MOD_BANDS = [(300, 800), (800, 2000), (2000, 5000), (5000, 12000)]
LEVEL_BANDS = [(300, 1000), (1000, 3000), (3000, 8000)]
FS = 48000


def riff(path):
    """(channels, bytes per sample, rate, float?, data) from a WAV's chunks — the `wave` module
    refuses WAVE_FORMAT_EXTENSIBLE, which the renderer writes for float samples."""
    with open(path, "rb") as f:
        b = f.read()
    i, fmt, data = 12, None, None
    while i + 8 <= len(b):
        cid, size = b[i:i + 4], int.from_bytes(b[i + 4:i + 8], "little")
        body = b[i + 8:i + 8 + size]
        if cid == b"fmt ":
            fmt = body
        elif cid == b"data":
            data = body
        i += 8 + size + (size & 1)
    tag = int.from_bytes(fmt[0:2], "little")
    if tag == 0xFFFE:  # extensible: the sub-format GUID's first two bytes
        tag = int.from_bytes(fmt[24:26], "little")
    ch = int.from_bytes(fmt[2:4], "little")
    fs = int.from_bytes(fmt[4:8], "little")
    bits = int.from_bytes(fmt[14:16], "little")
    return ch, bits // 8, fs, tag == 3, data


def load(path):
    """Mono float at 48 kHz (16/24-bit PCM or 32-bit float WAV), mean removed."""
    ch, sw, fs, is_float, raw = riff(path)
    if is_float and sw == 4:
        x = np.frombuffer(raw, "<f4").astype(float)
    elif is_float and sw == 8:
        x = np.frombuffer(raw, "<f8").astype(float)
    elif sw == 2:
        x = np.frombuffer(raw, "<i2").astype(float) / 32768
    elif sw == 3:
        b = np.frombuffer(raw, np.uint8).reshape(-1, 3)
        x = (b[:, 0].astype(np.int32) | (b[:, 1].astype(np.int32) << 8)
             | (b[:, 2].astype(np.int32) << 16))
        x = np.where(x >= 1 << 23, x - (1 << 24), x).astype(float) / (1 << 23)
    elif sw == 4:
        x = np.frombuffer(raw, "<i4").astype(float) / 2**31
    else:
        raise ValueError(f"{path}: {sw}-byte samples")
    x = x.reshape(-1, ch).mean(axis=1)
    if fs != FS:  # band-limited resampling by FFT
        X = np.fft.rfft(x)
        m = int(round(len(x) * FS / fs))
        Y = np.zeros(m // 2 + 1, complex)
        k = min(len(X), len(Y))
        Y[:k] = X[:k]
        x = np.fft.irfft(Y, m) * m / len(x)
    return x - x.mean()


def modulation(x):
    """Per MOD_BANDS: 100·2·|E(f*)|/E(0), f* the strongest envelope line in 12–400 Hz."""
    n = len(x)
    X = np.fft.fft(x)
    f = np.fft.fftfreq(n, 1 / FS)
    win = np.hanning(n)
    fe = np.fft.rfftfreq(n, 1 / FS)
    sel = (fe > 12) & (fe < 400)
    out = []
    for lo, hi in MOD_BANDS:
        e = np.abs(np.fft.ifft(np.where((f >= lo) & (f < hi), 2 * X, 0)))
        E = np.abs(np.fft.rfft(e * win)) / win.sum()
        out.append(100 * 2 * (E * sel).max() / E[0] if E[0] > 0 else 0.0)
    return out


def measure(x):
    W = 2 * FS
    wins = [x[i:i + W] for i in range(0, len(x) - W + 1, W)] or [x]
    mod = np.median([modulation(w) for w in wins], axis=0)
    P = np.abs(np.fft.rfft(x)) ** 2
    f = np.fft.rfftfreq(len(x), 1 / FS)
    ref = P[(f >= 50) & (f < 300)].sum()
    lev = [10 * np.log10(P[(f >= lo) & (f < hi)].sum() / ref) for lo, hi in LEVEL_BANDS]
    return mod, lev


def treble_mod(mod):
    """The 0.8–5 kHz figure: the mean of its two bands."""
    return 0.5 * (mod[1] + mod[2])


def self_test():
    # noise with its amplitude modulated by m at 100 Hz: the estimate scatters by ±0.02 on
    # 4 s of one noise, so its mean over eight noises is taken; it reads ≈ 8 % low (the band's
    # edges cut the sidebands of the components within 100 Hz of them: 2·100/1200), the same
    # for every signal compared, so within 15 % of m
    t = np.arange(4 * FS) / FS
    for m in (0.05, 0.1, 0.2):
        got = np.mean([
            modulation(np.random.default_rng(seed).standard_normal(len(t))
                       * (1 + m * np.cos(2 * np.pi * 100 * t)))[1] / 100
            for seed in range(8)
        ])
        assert abs(got / m - 1) < 0.15, f"m {m}: measured {got:.3f}"
    print("self-test ok")


def main(argv):
    if argv[:1] == ["--self-test"]:
        return self_test()
    if argv[:1] == ["--real"]:
        rows = []
        for p in sorted(glob.glob(os.path.join(argv[1], "*estable.wav"))):
            mod, lev = measure(load(p))
            rows.append([treble_mod(mod)] + list(mod) + lev)
        a = np.array(rows)
        for name, q in (("median", 50), ("p25", 25), ("p75", 75)):
            v = np.percentile(a, q, axis=0)
            print(f"{name:6s} n={len(a)} treble {v[0]:5.1f}%  mod " +
                  " ".join(f"{u:5.1f}" for u in v[1:5]) + "  levels " +
                  " ".join(f"{u:6.1f}" for u in v[5:]))
        return
    for p in argv:
        mod, lev = measure(load(p))
        print(f"{os.path.basename(p)[:40]:40s} treble {treble_mod(mod):5.1f}%  mod " +
              " ".join(f"{u:5.1f}" for u in mod) + "  levels " + " ".join(f"{u:6.1f}" for u in lev))


if __name__ == "__main__":
    main(sys.argv[1:])
