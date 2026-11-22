"""Channel models for test-vector generation, derived from the SPICE model in ../sim.

Board configuration modelled (matches the KiCad board on 2026-11-16):
  U1, U2 = TLV9062; L1, L2 = SDR0503-103JL (10 mH, 110 ohm); R8, R15 not fitted;
  JP1 bridged (inductive x11, 60.8 dB); JP2 open (capacitive x1).
ADC: PCM1808, 24-bit two's complement, full scale 3 Vpp centred on its 2.5 V internal bias.
"""
import sys, os, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.abspath(os.path.join(HERE, "..", "sim"))
sys.path.insert(0, SIM)
import common as C  # noqa: E402

FS_ADC_FULL = 3.0           # V peak-to-peak
CODE_FS = 2 ** 23           # 24-bit two's complement
COUNTS_PER_V = CODE_FS / (FS_ADC_FULL / 2)

IND_SUBS = {r"^R8  outind 0 100k": "R8  outind 0 1T"}                       # R8 DNP; JP1 bridged
CAP_SUBS = {r"^R15 outcap 0 100k": "R15 outcap 0 1T", r"^RJP2 g2b 0 0.01": "RJP2 g2b 0 1T"}  # R15 DNP; JP2 open
CAP_SUBS_X11 = {r"^R15 outcap 0 100k": "R15 outcap 0 1T"}                   # JP2 bridged


def _grid():
    return np.unique(np.concatenate([np.logspace(-1, 6.6, 3000), np.linspace(1, 60000, 6000)]))


def transfer(kind, cc="0.5p", jp2_open=True):
    """returns (freqs, complex H, noise PSD V^2/Hz at the ADC pin)"""
    old = os.getcwd()
    os.chdir(SIM)
    try:
        F = _grid()
        if kind == "ind":
            c = C.load("inductive.cir", "TLV9062", dict(IND_SUBS))
            H = c.ac(F, "vhum1", ["outind"])["outind"]
            psd, _ = C.noise(c, "outind", "TLV9062", F, ["xu1a", "xu1b"])
        else:
            subs = dict(CAP_SUBS if jp2_open else CAP_SUBS_X11)
            subs[r"^CC  lcd plate 0.5p"] = f"CC  lcd plate {cc}"
            c = C.load("capacitive.cir", "TLV9062", subs)
            H = c.ac(F, "vlcd", ["outcap"])["outcap"]
            psd, _ = C.noise(c, "outcap", "TLV9062", F, ["xu2a", "xu2b"], inject_in=("node",))
    finally:
        os.chdir(old)
    return F, H, psd


def interp_H(F, H, f):
    """interpolate a complex transfer function (magnitude in log, unwrapped phase) onto f"""
    f = np.asarray(f, float)
    mag = np.interp(f, F, np.log(np.maximum(np.abs(H), 1e-300)), left=-700, right=-700)
    ph = np.interp(f, F, np.unwrap(np.angle(H)))
    return np.exp(mag) * np.exp(1j * ph)


def adc_lpf(f, fs):
    """PCM1808-like decimation filter: flat to 0.454 fs, raised-cosine to zero at 0.5 fs (linear phase, delay omitted)"""
    f1, f2 = 0.454 * fs, 0.5 * fs
    g = np.ones_like(f)
    m = (f > f1) & (f < f2)
    g[m] = 0.5 * (1 + np.cos(np.pi * (f[m] - f1) / (f2 - f1)))
    g[f >= f2] = 0
    return g


def to_codes(v):
    """volts relative to the ADC's 2.5 V bias -> int32 codes, clipped like the ADC"""
    c = np.round(v * COUNTS_PER_V)
    return np.clip(c, -CODE_FS, CODE_FS - 1).astype(np.int32)


def shaped_noise(n, fs, F, psd, rng):
    """Gaussian noise of n samples whose one-sided PSD is psd(F) (V^2/Hz), band-limited like the ADC"""
    w = rng.standard_normal(n)
    X = np.fft.rfft(w)
    f = np.fft.rfftfreq(n, 1 / fs)
    S = np.interp(f, F, psd, left=psd[0], right=0)
    X *= np.sqrt(S * fs / 2) * adc_lpf(f, fs)
    return np.fft.irfft(X, n)


def write_wav24(path, fs, left, right):
    """stereo 24-bit little-endian PCM WAV (left = OUT_IND / VINL, right = OUT_CAP / VINR)"""
    import wave
    a = np.stack([left, right], axis=1).astype(np.int32).reshape(-1)
    b = a.astype("<i4").view(np.uint8).reshape(-1, 4)[:, :3].tobytes()
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(3)
        w.setframerate(int(fs))
        w.writeframes(b)


def read_wav24(path):
    import wave
    with wave.open(path, "rb") as w:
        fs = w.getframerate()
        n = w.getnframes()
        raw = np.frombuffer(w.readframes(n), dtype=np.uint8).reshape(-1, 3)
    v = (raw[:, 0].astype(np.int32) | (raw[:, 1].astype(np.int32) << 8) | (raw[:, 2].astype(np.int32) << 16))
    v = np.where(v & 0x800000, v - (1 << 24), v)
    v = v.reshape(-1, 2)
    return fs, v[:, 0], v[:, 1]
