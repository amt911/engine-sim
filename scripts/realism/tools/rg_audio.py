"""Audio IO + tach-less rpm tracking shared by the realism-gate prototype.

load():   mono float64 at FS (48 kHz) from 16/24/32-bit PCM or float WAV (incl. WAVE_FORMAT_EXTENSIBLE,
          which Python's `wave` refuses and the simulator writes), any rate (polyphase resampling).
track():  firing-frequency track by a harmonic comb with negative inter-teeth + Viterbi continuity
          (tach-less order tracking; cf. Leclere, Andre & Antoni, MSSP 81 (2016) 375-386 for the
          multi-order probabilistic idea; this is the plain dynamic-programming version).
"""
import numpy as np
from scipy import signal

FS = 48000


def riff(path):
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
    if tag == 0xFFFE:
        tag = int.from_bytes(fmt[24:26], "little")
    ch = int.from_bytes(fmt[2:4], "little")
    fs = int.from_bytes(fmt[4:8], "little")
    bits = int.from_bytes(fmt[14:16], "little")
    return ch, bits // 8, fs, tag == 3, data


def load(path, t0=None, t1=None, channel=None):
    """Mono float at FS. channel=None averages the channels (a wide stereo pair averaged combs a
    moving source slightly; pass 0/1 to take one microphone)."""
    ch, sw, fs, is_float, raw = riff(path)
    n = len(raw) // (sw * ch) * (sw * ch)
    raw = raw[:n]
    if is_float and sw == 4:
        x = np.frombuffer(raw, "<f4").astype(float)
    elif is_float and sw == 8:
        x = np.frombuffer(raw, "<f8").astype(float)
    elif sw == 2:
        x = np.frombuffer(raw, "<i2").astype(float) / 32768
    elif sw == 3:
        b = np.frombuffer(raw, np.uint8).reshape(-1, 3)
        x = (b[:, 0].astype(np.int32) | (b[:, 1].astype(np.int32) << 8) | (b[:, 2].astype(np.int32) << 16))
        x = np.where(x >= 1 << 23, x - (1 << 24), x).astype(float) / (1 << 23)
    elif sw == 4:
        x = np.frombuffer(raw, "<i4").astype(float) / 2 ** 31
    else:
        raise ValueError(f"{path}: {sw}-byte samples")
    x = x.reshape(-1, ch)
    x = x.mean(axis=1) if channel is None else x[:, min(channel, ch - 1)]
    if t0 is not None or t1 is not None:
        a = int((t0 or 0) * fs)
        b_ = int(t1 * fs) if t1 is not None else len(x)
        x = x[a:b_]
    if fs != FS:
        g = np.gcd(FS, fs)
        x = signal.resample_poly(x, FS // g, fs // g)
    return x - x.mean()


def stft_db(x, n=4096, hop=1024, nfft=None):
    """(t, f, power) of Hann frames."""
    nfft = nfft or n
    w = np.hanning(n)
    idx = np.arange(0, len(x) - n + 1, hop)
    fr = np.lib.stride_tricks.sliding_window_view(x, n)[idx] * w
    P = np.abs(np.fft.rfft(fr, nfft, axis=1)) ** 2
    return (idx + n / 2) / FS, np.fft.rfftfreq(nfft, 1 / FS), P


def track(x, fpr, rpm_lo, rpm_hi, n=None, hop=None, harmonics=None, jump_oct_per_s=1.5, fmax=6000.0,
          octave_guard_db=3.0, half_margin_db=6.0, return_margin=False, start_rpm=None, free_jump_db=12.0):
    """Firing-frequency track.

    fpr: firing events per crank revolution (4-stroke: cylinders / 2).
    Returns (t, rpm, salience): salience = mean comb contrast in dB of the chosen path (how far the
    firing harmonics stand over the half-way points between them). Frames with salience < ~3 dB are
    not an engine under load the comb can follow.
    """
    f0_lo = rpm_lo * fpr / 60.0
    if n is None:      # the window must resolve the comb: Hann main lobe (4 fs/n) under half the tooth spacing
        n = 4096 if f0_lo >= 90 else 8192 if f0_lo >= 45 else 16384
    hop = hop or n // 4
    if harmonics is None:
        harmonics = 8 if fpr >= 1.5 else 16
    # log spectrum up to fmax only, float32, built in chunks (a 5-minute recording would otherwise
    # need gigabytes for the 4x zero-padded frames)
    nfft = 4 * n
    f = np.fft.rfftfreq(nfft, 1 / FS)
    kmax = int(np.searchsorted(f, fmax + 0.3 * (2.0 * rpm_hi * fpr / 60.0))) + 8   # teeth + their quarter points
    f = f[:kmax]
    w = np.hanning(n)
    idx = np.arange(0, len(x) - n + 1, hop)
    t = (idx + n / 2) / FS
    L = np.empty((len(idx), kmax), dtype=np.float32)
    view = np.lib.stride_tricks.sliding_window_view(x, n)
    for a in range(0, len(idx), 512):
        fr = view[idx[a:a + 512]] * w
        L[a:a + 512] = 10 * np.log10(np.abs(np.fft.rfft(fr, nfft, axis=1)[:, :kmax]) ** 2 + 1e-20)
    # whiten: subtract a running median over ~1/3 octave so the comb reads contrast, not tilt
    df = f[1] - f[0]
    ext_lo, ext_hi = rpm_lo / 2.0, rpm_hi * 2.0     # octave images, scored for the margin only
    cands = np.exp(np.arange(np.log(ext_lo), np.log(ext_hi), np.log(1.004)))
    in_range = (cands >= rpm_lo) & (cands <= rpm_hi)
    f0 = cands * fpr / 60.0
    S = np.zeros((len(t), len(cands)))
    S_raw = np.zeros((len(t), len(cands)))
    m = np.arange(1, harmonics + 1)
    for j, ff in enumerate(f0):
        hf = m * ff
        ok = hf < fmax
        if ok.sum() < 2:
            continue
        on = np.round(hf[ok] / df).astype(int)
        q_lo = np.round((hf[ok] - 0.25 * ff) / df).astype(int)
        q_hi = np.round((hf[ok] + 0.25 * ff) / df).astype(int)
        h_lo = np.round((hf[ok] - 0.5 * ff) / df).astype(int)
        w = 1.0 / np.sqrt(m[ok])
        pk = np.maximum(np.maximum(L[:, on - 1], L[:, on]), L[:, on + 1])
        # contrast of each tooth over the quarter-way points (never an engine order of an even-firing
        # engine, so they read the noise floor; the half-way points may hold half orders)
        val = pk - 0.5 * (L[:, q_lo] + L[:, q_hi])
        sc = (val * w).sum(axis=1) / w.sum()
        S_raw[:, j] = sc      # the comb's plain contrast: gates frames without looking at half orders
        # sub-octave guard: at half the firing frequency the even teeth sit on the firing harmonics
        # and the odd teeth on the weaker half orders
        ev, od = (m[ok] % 2 == 0), (m[ok] % 2 == 1)
        if ev.sum() >= 2 and od.sum() >= 2:
            sc -= np.maximum(0.0, pk[:, ev].mean(axis=1) - pk[:, od].mean(axis=1) - octave_guard_db)
        # super-octave guard: at twice the firing frequency the half-way points hold the odd firing
        # harmonics, as strong as the teeth
        mid = np.maximum(np.maximum(L[:, h_lo - 1], L[:, h_lo]), L[:, h_lo + 1])
        sc -= np.maximum(0.0, mid.mean(axis=1) - pk.mean(axis=1) + half_margin_db)
        S[:, j] = sc
    S_full = S
    S = np.where(in_range[None, :], S_full, -1e6)   # the path stays inside [rpm_lo, rpm_hi]
    # Viterbi: transitions cost grows with |log2 ratio| beyond the allowed slew
    dt = hop / FS
    lg = np.log2(cands)
    max_step = jump_oct_per_s * dt
    cost = np.zeros_like(S)
    back = np.zeros(S.shape, dtype=int)
    cost[0] = S[0]
    if start_rpm is not None:   # the clip starts at a known speed (a render from idle): anchor the path there
        cost[0] = cost[0] - 200.0 * np.abs(np.log2(cands / start_rpm))
    # banded transitions for speed
    step = lg[1] - lg[0]
    band = max(1, int(np.ceil(4 * max_step / step)))
    offs = np.arange(-band, band + 1)
    pen = -np.maximum(0.0, np.abs(offs * step) - max_step) / max_step * 6.0  # dB per slew unit over
    for i in range(1, len(t)):
        best = np.full(len(cands), -1e18)
        arg = np.zeros(len(cands), dtype=int)
        for o, p in zip(offs, pen):
            src = np.arange(len(cands)) - o
            ok = (src >= 0) & (src < len(cands))
            v = np.full(len(cands), -1e18)
            v[ok] = cost[i - 1, src[ok]] + p
            upd = v > best
            best[upd] = v[upd]
            arg[upd] = src[upd]
        # allow a free jump (gear change) at a fixed price
        jmax = cost[i - 1].max() - free_jump_db
        jsrc = int(np.argmax(cost[i - 1]))
        upd = jmax > best
        best[upd] = jmax
        arg[upd] = jsrc
        cost[i] = best + S[i]
        back[i] = arg
    path = np.zeros(len(t), dtype=int)
    path[-1] = int(np.argmax(cost[-1]))
    for i in range(len(t) - 1, 0, -1):
        path[i - 1] = back[i, path[i]]
    rpm = cands[path]
    sal = S_raw[np.arange(len(t)), path]
    if return_margin:
        # how much better the chosen comb is than the same path one octave down / up
        k_oct = int(round(1.0 / step))
        ii = np.arange(len(t))
        alt = np.full(len(t), -1e9)
        for sh in (-k_oct, k_oct):
            j = path + sh
            ok = (j >= 0) & (j < len(cands))
            # best within +-2 % around the octave image
            for d in range(-5, 6):
                jj = np.clip(j + d, 0, len(cands) - 1)
                alt[ok] = np.maximum(alt[ok], S_full[ii[ok], jj[ok]])
        return t, rpm, sal, sal - alt
    return t, rpm, sal


def read_trk(path):
    """The project's .trk lines: '<t>s <rpm>rpm prom <dB>dB rms <dB>' -> (t centre, rpm, prom)."""
    rows = [l.split() for l in open(path) if l.strip()]
    T = np.array([float(r[0].rstrip("s")) for r in rows]) + 0.125
    R = np.array([float(r[1].rstrip("rpm")) for r in rows])
    P = np.array([float(r[3].rstrip("dB")) if r[3] != "nandB" else -99.0 for r in rows])
    return T, R, P


def unwrap_octaves(t, rpm, rpm_lo, rpm_hi, max_len_s=1.5):
    """Flip short stretches that sit one octave off both neighbours back (the comb's classic error)."""
    rpm = rpm.copy()
    lg = np.log2(rpm)
    cuts = [0] + [i for i in range(1, len(rpm)) if abs(lg[i] - lg[i - 1]) > 0.75] + [len(rpm)]
    for a, b in zip(cuts[:-1], cuts[1:]):
        if a == 0 or b == len(rpm) or t[b - 1] - t[a] > max_len_s:
            continue
        before, after, mid = lg[a - 1], lg[b], np.median(lg[a:b])
        for shift in (1.0, -1.0):
            if abs(mid + shift - before) < 0.35 and abs(mid + shift - after) < 0.35:
                new = rpm[a:b] * 2.0 ** shift
                if new.min() >= rpm_lo * 0.9 and new.max() <= rpm_hi * 1.1:
                    rpm[a:b] = new
    return rpm
