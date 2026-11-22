#!/usr/bin/env python3
"""Generate the test vectors (stereo 24-bit WAV, PCM1808 format) and their ground truth (JSON).

Left channel = OUT_IND (PCM1808 VINL), right channel = OUT_CAP (VINR).  Each file's ground truth
gives the true event times, the watch parameters and the answers a correct implementation should
produce.  Run:  python3 make_vectors.py OUTDIR
"""
import sys, os, json, math, pickle, time
import numpy as np
import channel as ch
import signals as S

OUT = sys.argv[1] if len(sys.argv) > 1 else "vectors"
os.makedirs(OUT, exist_ok=True)
TF = pickle.load(open(os.path.join(os.path.dirname(__file__), "tf.pkl"), "rb"))
RNG = np.random.default_rng(20261116)
FS = 48000
SPD = 86400.0
INH_STAGE_HZ = 8192.0       # divider stage at which inhibition removes oscillator pulses (Witschi's example)

MANIFEST = []


def hum(n, fs, a1, a3, f=60.0, rng=RNG):
    t = np.arange(n) / fs
    p1, p3 = rng.uniform(0, 2 * np.pi, 2)
    return a1 * np.sin(2 * np.pi * f * t + p1) + a3 * np.sin(2 * np.pi * 3 * f * t + p3)


def save(name, fs, L, R, truth):
    path = os.path.join(OUT, name + ".wav")
    ch.write_wav24(path, fs, L.codes(), R.codes())
    peakL = int(np.abs(L.codes()).max()); peakR = int(np.abs(R.codes()).max())
    truth.update({"file": name + ".wav", "fs_hz": fs, "duration_s": L.n / fs,
                  "peak_codes": {"left": peakL, "right": peakR},
                  "clipped": {"left": bool(peakL >= ch.CODE_FS - 1), "right": bool(peakR >= ch.CODE_FS - 1)}})
    json.dump(truth, open(os.path.join(OUT, name + ".json"), "w"), indent=1)
    MANIFEST.append({"file": name + ".wav", "truth": name + ".json", "what": truth["description"]})
    print(f"{name}: {L.n/fs:.1f} s, peak L {peakL} R {peakR}")


def blank(fs, dur, kind):
    """a channel carrying only its own noise (the other sensor sees no watch)"""
    s = S.Synth(fs, int(dur * fs), TF[kind])
    s.add_noise(RNG)
    return s


# ------------------------------------------------------------------ motor pulse trains
def motor_train(name, desc, dur, amp, eps=0.0, step=1.0, t0=0.37, pw=6.8e-3, duty=0.75,
                inh_period_s=None, inh_n=0, inh_first_s=None, hum_v=(0.0, 0.0), duties=None, pws=None,
                fs=FS):
    L = S.Synth(fs, int(dur * fs), TF["ind"])
    events = []
    delay = 0.0
    inh_events = []
    k = 0
    t_nom = t0
    while True:
        t = t0 + k * step / (1 + eps)
        if inh_period_s:
            # inhibition events happen at watch times inh_first_s + j * inh_period_s (between steps)
            tw = k * step                              # watch time of this step since t0
            j_done = sum(1 for e in inh_events)
            while inh_first_s + len(inh_events) * inh_period_s <= tw:
                when_watch = inh_first_s + len(inh_events) * inh_period_s
                inh_events.append({"watch_time_s": when_watch,
                                   "true_time_s": t0 + when_watch / (1 + eps) + delay,
                                   "delay_added_s": inh_n / INH_STAGE_HZ})
                delay += inh_n / INH_STAGE_HZ
        t += delay
        if t > dur - 0.25:
            break
        d = duties[k % len(duties)] if duties else duty
        w = pws[k % len(pws)] if pws else pw
        pol = 1 if k % 2 == 0 else -1
        L.add_event(lambda tr, a=amp * pol, w=w, d=d: S.motor_emf(tr, a, pw=w, duty=d), t)
        events.append({"index": k, "t_onset_s": round(t, 9), "polarity": pol, "pulse_width_s": w, "duty": d})
        k += 1
    L.add_noise(RNG)
    if hum_v[0] or hum_v[1]:
        L.y += hum(L.n, fs, *hum_v)
    R = blank(fs, dur, "cap")
    T_true = step / (1 + eps)
    truth = {"description": desc, "channel": "left (OUT_IND, inductive)", "signal": "stepper motor drive pulses",
             "assumed_board": {"jp1": "bridged (x11, 60.8 dB)", "jp2": "open (x1)", "r8_r15": "not fitted"},
             "pickup_emf_peak_v": amp, "noise_rms_codes_left": 10215,
             "hum_at_adc_v": {"60Hz": hum_v[0], "180Hz": hum_v[1]},
             "nominal_step_interval_s": step, "events": events,
             "expected": {"rate_without_inhibition_s_per_day": round(SPD * eps, 6)}}
    if inh_period_s:
        n_per_cycle = inh_period_s / step
        T_avg = (n_per_cycle * step / (1 + eps) + inh_n / INH_STAGE_HZ) / n_per_cycle
        truth["inhibition"] = {"period_s": inh_period_s, "oscillator_pulses_removed": inh_n,
                               "removal_stage_hz": INH_STAGE_HZ,
                               "delay_per_event_us": round(inh_n / INH_STAGE_HZ * 1e6, 3),
                               "events": inh_events}
        truth["expected"]["rate_with_inhibition_s_per_day"] = round(SPD * (step / T_avg - 1), 6)
        truth["expected"]["inhibition_period_s"] = inh_period_s
    else:
        truth["expected"]["rate_with_inhibition_s_per_day"] = round(SPD * eps, 6)
        truth["expected"]["inhibition_period_s"] = None
    truth["expected"]["note"] = ("rate = 86400 * (nominal interval / measured mean interval - 1); positive = watch gains. "
                                 "Measured timestamps carry a constant delay (analogue chain about 7 us plus the "
                                 "ADC's digital filter); it cancels out of every rate.")
    save(name, fs, L, R, truth)


# ------------------------------------------------------------------ LCD
def lcd(name, desc, dur, pattern, pattern_hz, eps, cc="0.5p", v=3.0, hum_v=(0.0, 0.0), x11=False, fs=FS):
    key = ("cap_x11" if x11 else "cap") if cc == "0.5p" else None
    tf = TF[key] if key else ch.transfer("cap", cc=cc, jp2_open=not x11)
    R = S.Synth(fs, int(dur * fs), tf)
    slots = len(pattern)
    T = 1.0 / pattern_hz / (1 + eps)
    t0 = 0.0123
    edges = []
    level = pattern[-1]
    n_per = int(dur / T) + 2
    for p in range(n_per):
        for s, lv in enumerate(pattern):
            t = t0 + (p + s / slots) * T
            if t > dur - 0.05:
                break
            dv = (lv - level) * v
            level = lv
            if dv:
                R.add_event(lambda tr, dv=dv: dv * ((tr >= 0) & (tr < 0.035)), t, pre=1e-3, post=0.04, keep=0.03)
                edges.append({"t_s": round(t, 9), "dv": round(dv, 4)})
    R.add_noise(RNG)
    if hum_v[0] or hum_v[1]:
        R.y += hum(R.n, fs, *hum_v)
    L = blank(fs, dur, "ind")
    truth = {"description": desc, "channel": "right (OUT_CAP, capacitive)", "signal": "LCD drive",
             "assumed_board": {"jp1": "bridged (x11)", "jp2": "bridged (x11)" if x11 else "open (x1)", "r8_r15": "not fitted"},
             "display_coupling_pf": float(cc.rstrip("p")), "electrode_swing_v": v,
             "pattern_levels_fraction_of_swing": pattern, "pattern_repetition_hz_nominal": pattern_hz,
             "hum_at_adc_v": {"60Hz": hum_v[0], "180Hz": hum_v[1]},
             "edges": edges[:400], "edges_truncated": len(edges) > 400, "edge_count": len(edges),
             "expected": {"rate_s_per_day": round(SPD * eps, 6), "pattern_repetition_hz_nominal": pattern_hz,
                          "note": "LCD rate is the oscillator rate; any inhibition applied after the display divider is not visible."}}
    save(name, fs, L, R, truth)


# ------------------------------------------------------------------ tones
def tone(name, desc, dur, f0_nom, eps, amp_v, side, kind, harmonics=(), hum_v=(0.0, 0.0), fs=FS):
    A = S.Synth(fs, int(dur * fs), TF[kind])
    A.add_tone(f0_nom * (1 + eps), amp_v, phase=RNG.uniform(0, 2 * np.pi), harmonics=harmonics)
    A.add_noise(RNG)
    if hum_v[0] or hum_v[1]:
        A.y += hum(A.n, fs, *hum_v)
    B = blank(fs, dur, "cap" if side == "left" else "ind")
    L, R = (A, B) if side == "left" else (B, A)
    truth = {"description": desc, "channel": side, "signal": f"tone {f0_nom} Hz",
             "nominal_frequency_hz": f0_nom, "true_frequency_hz": f0_nom * (1 + eps), "amplitude_at_adc_v": amp_v,
             "expected": {"rate_s_per_day": round(SPD * eps, 6)}}
    save(name, fs, L, R, truth)


if __name__ == "__main__":
    t_start = time.time()
    # ---- single pulses at several amplitudes (3 pulses each, alternating polarity)
    for amp, tag in ((20e-6, "20uV"), (100e-6, "100uV"), (1e-3, "1mV"), (2.5e-3, "2p5mV_clipping")):
        motor_train(f"ind_pulses_{tag}", f"Three motor pulses, pickup EMF {amp*1e6:.0f} uV peak, 6.8 ms chopped at 75 %"
                    + (" (overdrives the ADC: shows clipping)" if amp > 1.5e-3 else ""),
                    3.2, amp, t0=0.5)
    # ---- drive-level sweep: duty 37.5 % to 100 %, two pulse widths
    duties = [0.375, 0.5, 0.625, 0.75, 0.875, 1.0, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0]
    pws = [4.9e-3] * 6 + [7.8e-3] * 6
    motor_train("ind_duty_sweep", "Twelve pulses: chopping duty 37.5 to 100 % at pulse widths 4.9 ms and 7.8 ms, EMF 100 uV",
                12.6, 100e-6, t0=0.4, duties=duties, pws=pws)
    # ---- rate tests
    motor_train("ind_rate_no_inhibition", "1 Hz stepping, no inhibition, watch losing 0.432 s/d, EMF 50 uV, 3 mV 60 Hz hum",
                31.0, 50e-6, eps=-5e-6, t0=0.61, hum_v=(3e-3, 1e-3))
    motor_train("ind_inhibition_10s", "1 Hz stepping, crystal +14 ppm, inhibition every 10 s removing 1 pulse at 8192 Hz, EMF 50 uV, hum",
                25.5, 50e-6, eps=14e-6, t0=0.29, inh_period_s=10.0, inh_n=1, inh_first_s=3.5, hum_v=(3e-3, 1e-3))
    motor_train("ind_inhibition_60s", "1 Hz stepping, crystal +10.453 ppm, inhibition every 60 s removing 4 pulses at 8192 Hz, EMF 30 uV, hum",
                130.5, 30e-6, eps=10.453e-6, t0=0.44, inh_period_s=60.0, inh_n=4, inh_first_s=17.5, hum_v=(3e-3, 1e-3))
    motor_train("ind_step_20s", "Watch without seconds hand: one step every 20 s, rate +0.5 s/d, EMF 100 uV",
                63.0, 100e-6, eps=0.5 / SPD, step=20.0, t0=1.7)
    # ---- tuning fork (Accutron-type 360 Hz) on the inductive channel
    tone("ind_tuning_fork_360Hz", "Tuning-fork watch, 360 Hz, +6 ppm (+0.518 s/d), 80 mV at the ADC with harmonics, hum",
         10.0, 360.0, 6e-6, 0.08, "left", "ind", harmonics=((2, 0.3), (3, 0.15)), hum_v=(3e-3, 1e-3))
    # ---- LCD
    lcd("lcd_static_64Hz", "Static-drive LCD, 64 Hz square, coupling 0.5 pF, JP2 open, watch +3 ppm (+0.259 s/d), 1 mV hum",
        20.0, [0, 1], 64.0, 3e-6, hum_v=(1e-3, 0.3e-3))
    lcd("lcd_multiplex_32Hz", "Multiplexed LCD (1/3 duty, 1/3 bias style), 64 Hz frame with inversion, pattern repeats at 32 Hz, "
        "coupling 0.2 pF, watch -8 ppm (-0.691 s/d)",
        20.0, [2/3, 1/3, 1/3, 0, 1/3, 1/3], 32.0, -8e-6, cc="0.2p", hum_v=(1e-3, 0.3e-3))
    lcd("lcd_weak_static_64Hz", "Weak coupling (0.05 pF), static 64 Hz, JP2 open, watch +1 ppm (+0.086 s/d), 2 mV hum",
        20.0, [0, 1], 64.0, 1e-6, cc="0.05p", hum_v=(2e-3, 0.5e-3))
    # ---- crystal, 96 kHz sampling, JP2 bridged, plastic case: 1 V oscillator swing through 2 fF
    g = abs(ch.interp_H(TF["cap_x11_2f"][0], TF["cap_x11_2f"][1], [32768.0])[0])
    tone("xtal_32768Hz_96k", "Quartz crystal through a plastic case: 32768 Hz +10.453 ppm (raw crystal, before inhibition), "
         "1 V swing coupled through 2 fF, JP2 bridged, ADC at 96 kHz",
         4.0, 32768.0, 10.453e-6, 1.0 * g, "right", "cap_x11_2f", hum_v=(0.5e-3, 0.0), fs=96000)
    json.dump(MANIFEST, open(os.path.join(OUT, "manifest.json"), "w"), indent=1)
    print(f"done in {time.time()-t_start:.0f} s")
