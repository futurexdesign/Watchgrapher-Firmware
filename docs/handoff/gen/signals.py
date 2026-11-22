"""Watch signal models and the ADC-rate synthesis of what the head delivers to the PCM1808.

Motor: coil R = 2 kOhm, L = 1 H, drive 1.55 V, pulse chopped at 1024 Hz (period 976.5625 us).
The sense-coil EMF is proportional to the voltage across the motor inductance (L dI/dt), scaled so
that its first peak equals `amp` volts.  All of this is an assumption about a typical movement.
"""
import math
import numpy as np
import channel as ch

TCHOP = 1 / 1024.0


def motor_segments(pw, duty, tchop=TCHOP):
    """list of (t_start, on) for one drive pulse starting at t = 0; ends with an 'off' segment"""
    segs = []
    if pw <= 0:
        return [(0.0, False)]
    if duty >= 0.999:
        return [(0.0, True), (pw, False)]
    t = 0.0
    while t < pw - 1e-12:
        segs.append((t, True))
        t_off = min(t + duty * tchop, pw)
        segs.append((t_off, False))
        t += tchop
    return segs


def motor_emf(trel, amp, pw=6.8e-3, duty=0.75, R=2000.0, L=1.0, V=1.55):
    """pickup EMF (V) at times trel (s, relative to the drive onset); zero before onset"""
    tau = L / R
    segs = motor_segments(pw, duty)
    ts = np.array([s[0] for s in segs])
    on = np.array([s[1] for s in segs])
    # current at each segment start
    I0 = np.zeros(len(segs))
    for k in range(1, len(segs)):
        dt = ts[k] - ts[k - 1]
        Ifin = V / R if on[k - 1] else 0.0
        I0[k] = Ifin + (I0[k - 1] - Ifin) * math.exp(-dt / tau)
    out = np.zeros_like(trel, dtype=float)
    m = trel >= 0
    k = np.searchsorted(ts, trel[m], side="right") - 1
    Ifin = np.where(on[k], V / R, 0.0)
    I = Ifin + (I0[k] - Ifin) * np.exp(-(trel[m] - ts[k]) / tau)
    VL = np.where(on[k], V, 0.0) - R * I
    out[m] = amp * VL / V
    return out


class Synth:
    """adds band-limited, filtered signals onto an ADC-rate record"""
    OS = 32  # oversampling of the internal grid relative to the ADC rate

    def __init__(self, fs, n, tf):
        self.fs, self.n = fs, n
        self.F, self.H, self.psd = tf
        self.y = np.zeros(n)             # volts at the ADC pin, relative to its 2.5 V bias

    def add_event(self, fn, t0, pre=2e-3, post=0.2, keep=None):
        """fn(trel) gives the input waveform (EMF or electrode voltage) around an event at t0 seconds.
        keep: only add output up to t0 + keep (used for steps, whose window is closed by a return edge)"""
        fsh = self.fs * self.OS
        n0 = int(math.floor((t0 - pre) * self.fs))        # first ADC sample of the window
        nw = int(math.ceil((pre + post) * self.fs))
        m = nw * self.OS
        th = (n0 * self.OS + np.arange(m)) / fsh           # absolute hi-res times
        x = fn(th - t0)
        X = np.fft.rfft(x)
        f = np.fft.rfftfreq(m, 1 / fsh)
        X *= ch.interp_H(self.F, self.H, f) * ch.adc_lpf(f, self.fs)
        yh = np.fft.irfft(X, m)
        ya = yh[:: self.OS]
        a, b = max(n0, 0), min(n0 + nw, self.n)
        if keep is not None:
            b = min(b, int(math.floor((t0 + keep) * self.fs)))
        if a < b:
            self.y[a:b] += ya[a - n0: b - n0]

    def add_tone(self, f0, amp_v, phase=0.0, harmonics=()):
        """sinusoid (and optional harmonics: list of (k, rel_amp)) at the ADC pin after the channel response"""
        t = np.arange(self.n) / self.fs
        g = abs(ch.interp_H(self.F, self.H, [f0])[0])
        self.y += amp_v * np.sin(2 * np.pi * f0 * t + phase)
        for k, ra in harmonics:
            gk = abs(ch.interp_H(self.F, self.H, [k * f0])[0]) / g * ch.adc_lpf(np.array([k * f0]), self.fs)[0]
            self.y += amp_v * ra * gk * np.sin(2 * np.pi * k * f0 * t + k * phase)

    def add_noise(self, rng, scale=1.0):
        self.y += scale * ch.shaped_noise(self.n, self.fs, self.F, self.psd, rng)

    def codes(self):
        return ch.to_codes(self.y)
