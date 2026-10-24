#!/usr/bin/env python3
"""Compare renders with reference recordings by two numbers per octave band:
- tilt: band energy in dB relative to 250-500 Hz (a muffled road car falls ~10-30 dB by 4-8 kHz);
- contrast: 98th percentile over median of the band's spectrum, dB (harmonic lines over a dark
  floor read high; pure noise reads 7.5 dB, the ratio for exponentially distributed bins).
Usage: scripts/realism.py file.wav@t0 [more.wav@t0 ...]   (t0: start of the 1 s window, s)
       scripts/realism.py --self-test
Reads 16-bit PCM and 32-bit float WAV, including WAVE_FORMAT_EXTENSIBLE (hound's float WAVs)."""
import sys
import numpy as np

BANDS = [(125, 250), (250, 500), (500, 1000), (1000, 2000), (2000, 4000), (4000, 8000)]

def load(path):
    b = open(path, "rb").read()
    i, fs, data, fmt, bits = 12, None, None, None, None
    while i < len(b) - 8:
        cid, n = b[i:i + 4], int.from_bytes(b[i + 4:i + 8], "little")
        if cid == b"fmt ":
            fmt = int.from_bytes(b[i + 8:i + 10], "little")
            fs = int.from_bytes(b[i + 12:i + 16], "little")
            bits = int.from_bytes(b[i + 22:i + 24], "little")
        if cid == b"data":
            raw = b[i + 8:i + 8 + n]
            # 3 = IEEE float; 0xFFFE = extensible, whose sub-format hound sets to float at 32 bits
            is_float = fmt == 3 or (fmt == 0xFFFE and bits == 32)
            data = np.frombuffer(raw, dtype=np.float32 if is_float else np.int16).astype(float)
            if not is_float:
                data /= 32768.0
        i += 8 + n + (n & 1)
    return fs, data

def measure(x, fs, t0, dur=1.0):
    seg = x[int(t0 * fs):int((t0 + dur) * fs)]
    seg = seg * np.hanning(len(seg))
    p = np.abs(np.fft.rfft(seg)) ** 2
    f = np.fft.rfftfreq(len(seg), 1 / fs)
    e = [p[(f >= lo) & (f < hi)].sum() for lo, hi in BANDS]
    tilt = [10 * np.log10(v / e[1]) for v in e]
    contrast = []
    for lo, hi in BANDS:
        b = p[(f >= lo) & (f < hi)]
        contrast.append(10 * np.log10(np.percentile(b, 98) / np.median(b)))
    return tilt, contrast

def self_test():
    # a 100 Hz harmonic series written as float32 extensible must read back as itself
    import struct, tempfile, os
    fs = 48000
    t = np.arange(fs) / fs
    x = sum(np.sin(2 * np.pi * 100 * k * t) / k for k in range(1, 20)).astype(np.float32) * 0.1
    data = x.tobytes()
    fmt = struct.pack("<HHIIHH", 0xFFFE, 1, fs, fs * 4, 4, 32)
    blob = b"WAVE" + b"fmt " + struct.pack("<I", len(fmt)) + fmt + b"data" + struct.pack("<I", len(data)) + data
    path = os.path.join(tempfile.mkdtemp(), "t.wav")
    open(path, "wb").write(b"RIFF" + struct.pack("<I", len(blob)) + blob)
    rfs, y = load(path)
    assert rfs == fs and np.allclose(y, x, atol=1e-7), "float extensible WAV misread"
    _, contrast = measure(y, fs, 0.0)
    assert contrast[1] > 30, f"a harmonic series must read as lines, got {contrast[1]:.1f} dB"
    print("self-test ok")

if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        self_test()
        sys.exit(0)
    print("bands (Hz): " + "  ".join(f"{lo}-{hi}" for lo, hi in BANDS))
    for spec in sys.argv[1:]:
        path, t0 = spec.rsplit("@", 1)
        fs, x = load(path)
        tilt, contrast = measure(x, fs, float(t0))
        name = path.split("/")[-1]
        print(f"{name:28s} tilt     " + " ".join(f"{v:6.1f}" for v in tilt))
        print(f"{'':28s} contrast " + " ".join(f"{v:6.1f}" for v in contrast))
