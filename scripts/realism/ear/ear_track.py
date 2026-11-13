"""Phase-locked order tracking on the firing fundamental (V10: 10 firings per 720 deg).
angle(x, fs, rpm_fn, fpc) -> (y, S, rpm_mean): x resampled to S samples per engine cycle."""
import numpy as np
from scipy import signal
from scipy.interpolate import interp1d

def rough_rpm(x, fs, fpc, rlo, rhi, hop=0.05):
    """Firing-fundamental peak per frame in [rlo, rhi] rpm -> (t, rpm)."""
    n = 8192
    f, t, Z = signal.stft(x, fs, nperseg=n, noverlap=n - int(hop*fs))
    P = np.abs(Z)**2
    flo, fhi = fpc*rlo/120, fpc*rhi/120
    b = (f >= flo) & (f <= fhi)
    k = np.argmax(P[b], 0)
    fr = f[b][k]
    return t, fr*120/fpc

def fine_phase(x, fs, t_r, rpm_r, fpc, bw=25.0):
    """Firing phase (cycles of the firing fundamental) at every sample."""
    tt = np.arange(len(x))/fs
    f0 = np.interp(tt, t_r, signal.medfilt(rpm_r, 5))*fpc/120.0
    ph0 = np.cumsum(f0)/fs
    z = x*np.exp(-2j*np.pi*ph0)
    sos = signal.butter(4, bw, fs=fs, output="sos")
    zl = signal.sosfiltfilt(sos, z.real) + 1j*signal.sosfiltfilt(sos, z.imag)
    res = np.unwrap(np.angle(zl))/(2*np.pi)
    mag = np.abs(zl)
    return ph0 + res, mag

def angle(x, fs, ph_fire, fpc, S):
    """Resample x on a uniform grid of engine-cycle phase (fpc firings per cycle)."""
    cyc = ph_fire/fpc
    sos = signal.butter(8, 16000, fs=fs, output="sos")
    xl = signal.sosfiltfilt(sos, x)
    c0, c1 = np.ceil(cyc[0]), np.floor(cyc[-1])
    grid = np.arange(c0, c1, 1.0/S)
    y = np.interp(grid, cyc, xl)
    # local rpm per cycle
    tt = np.arange(len(x))/fs
    tc = np.interp(np.arange(c0, c1+1), cyc, tt)
    rpm_c = 120.0/np.diff(tc)
    return y, rpm_c
