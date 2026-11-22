#!/usr/bin/env python3
"""Sample-waveform figures for FIRMWARE_HANDOFF.md, drawn from the test vectors."""
import sys, os, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gen"))
import channel as ch
import timing_ref as R

VD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "vectors")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
os.makedirs(OUT, exist_ok=True)
C1, C2, C3, INK, INK2, GRID = "#2a78d6", "#eb6834", "#1baf7a", "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.2,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.dpi": 140})


def load(name):
    fs, L, Rr = ch.read_wav24(os.path.join(VD, name + ".wav"))
    return fs, L.astype(float), Rr.astype(float), json.load(open(os.path.join(VD, name + ".json")))


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)


# 1 overview: 1 Hz pulse train
fs, L, _, t = load("ind_rate_no_inhibition")
n = int(5 * fs)
fig, ax = plt.subplots(figsize=(8, 2.6))
ax.plot(np.arange(n) / fs, L[:n] / 1e6, color=C1, lw=0.6)
ax.set_xlabel("time (s)"); ax.set_ylabel("ADC codes (millions)")
ax.set_title("Inductive channel, 1 Hz quartz movement, 50 µV pickup EMF, with 60/180 Hz hum (raw codes)", loc="left")
save(fig, "01_pulse_train_overview.png")

# 2 single pulse at three amplitudes
fig, axs = plt.subplots(3, 1, figsize=(8, 6.4), sharex=True)
for ax, name, lab in zip(axs, ["ind_pulses_20uV", "ind_pulses_100uV", "ind_pulses_1mV"], ["20 µV", "100 µV", "1 mV"]):
    fs, L, _, t = load(name)
    y = R.highpass(L, fs)
    t0 = t["events"][0]["t_onset_s"]
    a, b = int((t0 - 2e-3) * fs), int((t0 + 10e-3) * fs)
    ax.plot((np.arange(a, b) / fs - t0) * 1e3, y[a:b] / 1e3, color=C1, lw=0.8)
    ax.set_ylabel("k codes")
    ax.set_title(f"pickup EMF {lab} peak (×11 gain, after 400 Hz high-pass)", loc="left")
axs[-1].set_xlabel("time from drive onset (ms)   drive 6.8 ms, chopped 1024 Hz at 75 %")
save(fig, "02_single_pulse_three_amplitudes.png")

# 3 leading edge zoom with samples
fs, L, _, t = load("ind_pulses_100uV")
y = R.highpass(L, fs)
t0 = t["events"][0]["t_onset_s"]
a, b = int((t0 - 0.4e-3) * fs), int((t0 + 1.2e-3) * fs)
tt = (np.arange(a, b) / fs - t0) * 1e3
fig, ax = plt.subplots(figsize=(8, 3.2))
ax.plot(tt, y[a:b] / 1e3, color=C1, lw=1)
ax.plot(tt, y[a:b] / 1e3, "o", color=C1, ms=4)
i0 = int(t0 * fs)
pk = np.max(np.abs(y[i0:i0 + 5]))  # first (leading) spike only
ax.axhline(0.5 * pk / 1e3, color=C2, lw=1, ls="--")
ax.text(-0.38, 0.5 * pk / 1e3, "50 % of the leading spike", color=INK2, va="bottom", ha="left")
ax.axvline(0, color=INK2, lw=0.8, ls=":")
ax.set_xlabel("time from true drive onset (ms); dots = 48 kHz samples (20.8 µs apart)")
ax.set_ylabel("k codes")
ax.set_title("Leading edge, 100 µV EMF: the edge is shorter than one sample, so timestamps must interpolate", loc="left")
save(fig, "03_leading_edge_samples.png")

# 4 duty sweep
fs, L, _, t = load("ind_duty_sweep")
y = R.highpass(L, fs)
fig, axs = plt.subplots(3, 1, figsize=(8, 5.6), sharex=True)
for ax, k in zip(axs, [6, 9, 11]):
    e = t["events"][k]
    t0 = e["t_onset_s"]
    a, b = int((t0 - 1e-3) * fs), int((t0 + 10e-3) * fs)
    ax.plot((np.arange(a, b) / fs - t0) * 1e3, y[a:b] / 1e3, color=C1, lw=0.8)
    ax.set_ylabel("k codes")
    ax.set_title(f"drive {e['pulse_width_s']*1e3:.1f} ms, chopping duty {e['duty']*100:.1f} %, polarity {e['polarity']:+d}", loc="left")
axs[-1].set_xlabel("time from drive onset (ms)  - each chop gives an on-spike and an opposite off-spike")
save(fig, "04_duty_sweep.png")

# 5 inhibition residual sawtooth
fig, axs = plt.subplots(2, 1, figsize=(8, 5.2))
for ax, name in zip(axs, ["ind_inhibition_10s", "ind_inhibition_60s"]):
    fs, L, _, t = load(name)
    pulses, y, _ = R.detect_motor_pulses(L, fs)
    ts = R.timestamp_pulses(pulses, y, fs)
    nn = np.round(ts - ts[0])
    p = np.polyfit(nn, ts, 1)
    ax.plot(ts, (ts - np.polyval(p, nn)) * 1e6, "o-", color=C1, ms=3.5, lw=0.8)
    for ev in t["inhibition"]["events"]:
        ax.axvline(ev["true_time_s"], color=C2, lw=0.8, ls="--")
    ax.set_ylabel("residual (µs)")
    inh = t["inhibition"]
    ax.set_title(f"{name}: residual from one straight line; jumps of {inh['delay_per_event_us']:.0f} µs every {inh['period_s']:.0f} s (dashed)", loc="left")
axs[-1].set_xlabel("time (s)")
save(fig, "05_inhibition_sawtooth.png")

# 6 LCD
fig, axs = plt.subplots(2, 1, figsize=(8, 5))
for ax, name, ttl in zip(axs, ["lcd_static_64Hz", "lcd_multiplex_32Hz"], ["Static LCD, 64 Hz drive, 0.5 pF coupling", "Multiplexed LCD, 64 Hz frame, pattern repeats at 32 Hz, 0.2 pF"]):
    fs, _, Rr, t = load(name)
    y = R.highpass(Rr, fs, fc=10.0)
    a, b = int(0.5 * fs), int(0.5 * fs + 0.07 * fs)
    ax.plot((np.arange(a, b) / fs - 0.5) * 1e3, y[a:b] / 1e3, color=C3, lw=0.8)
    ax.set_ylabel("k codes")
    ax.set_title(ttl, loc="left")
axs[-1].set_xlabel("time (ms)  - capacitive channel, ×1: one spike per electrode transition")
save(fig, "06_lcd_waveforms.png")

# 7 crystal spectrum + tuning fork
fig, axs = plt.subplots(1, 2, figsize=(8, 3))
fs, _, Rr, t = load("xtal_32768Hz_96k")
seg = Rr[: int(fs)] * np.hanning(int(fs))
X = 20 * np.log10(np.abs(np.fft.rfft(seg)) / (np.sum(np.hanning(int(fs))) / 2) + 1e-3)
f = np.fft.rfftfreq(int(fs), 1 / fs)
m = (f > 30000) & (f < 35500)
axs[0].plot(f[m] / 1e3, X[m], color=C3, lw=0.7)
axs[0].set_xlabel("frequency (kHz)"); axs[0].set_ylabel("dB codes")
axs[0].set_title("Crystal, 96 kHz sampling, 1 s FFT", loc="left")
fs, L, _, t = load("ind_tuning_fork_360Hz")
a, b = int(1 * fs), int(1.02 * fs)
axs[1].plot((np.arange(a, b) / fs - 1) * 1e3, L[a:b] / 1e3, color=C1, lw=0.9)
axs[1].set_xlabel("time (ms)"); axs[1].set_ylabel("k codes")
axs[1].set_title("Tuning fork 360 Hz, inductive", loc="left")
save(fig, "07_crystal_and_tuning_fork.png")
print(sorted(os.listdir(OUT)))
