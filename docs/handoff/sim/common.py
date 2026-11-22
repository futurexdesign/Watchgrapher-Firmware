import math, re
import numpy as np
import spicelite as S

KT4 = 4 * S.KB * S.TEMP


def load(path, opamp="TLV9062", subs=None):
    t = open(path).read()
    t = re.sub(r"\b(TLV9062|OPA2376)\s*$", opamp, t, flags=re.M)
    for a, b in (subs or {}).items():
        t2 = re.sub(a, b, t, flags=re.M)
        assert t2 != t or re.search(a, t, flags=re.M), a
        t = t2
    c = S.Circuit(text=t)
    c.op()
    return c


def en_psd(opamp, f):
    """op amp voltage noise density squared, V^2/Hz"""
    if opamp == "TLV9062":
        return (10e-9) ** 2 * (1 + 1560.0 / f)     # 16 nV @ 1 kHz, 10 nV @ 10 kHz (TI datasheet)
    return (7.5e-9) ** 2 * (1 + 100.0 / f)          # OPA2376: 7.5 nV broadband; 1/f corner assumed


def in_psd(opamp, f):
    return (23e-15) ** 2 if opamp == "TLV9062" else (2e-15) ** 2


def noise(c, out, opamp, freqs, opamps, inject_in=()):
    """output noise PSD (V^2/Hz) and per-source breakdown"""
    parts = {}
    tot = np.zeros(len(freqs))
    for r in c.resistors():
        if r[4] < 1 or r[4] > 1e11:        # skip jumper shorts / opens
            continue
        if r[1].split(".")[0].startswith("x"):
            continue                        # op amp internals
        h = c.ac_inject(freqs, (r[2], r[3]), out)
        p = KT4 / r[4] * np.abs(h) ** 2
        parts[r[1]] = p
        tot += p
    for u in opamps:
        h = c.ac(freqs, f"{u}.vn", [out])[out]
        p = en_psd(opamp, freqs) * np.abs(h) ** 2
        parts[u + " en"] = p
        tot += p
    for node in inject_in:
        h = c.ac_inject(freqs, ("0", node), out)
        p = in_psd(opamp, freqs) * np.abs(h) ** 2
        parts["in@" + node] = p
        tot += p
    return tot, parts


def band(freqs, mag, ref=None):
    ref = ref if ref is not None else mag.max()
    above = np.where(mag >= ref / math.sqrt(2))[0]
    return freqs[above[0]], freqs[above[-1]]


def integ(freqs, psd, f1, f2):
    m = (freqs >= f1) & (freqs <= f2)
    return math.sqrt(np.trapezoid(psd[m], freqs[m]))
