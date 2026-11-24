# Generated tick scenarios

Synthetic mechanical-watch recordings with exact ground truth, for measuring the
timing chain (`docs/plans/TIMING_ENGINE_PLAN.md`, Phase 0, task 0a). Each
`<name>.wav` (mono, 16-bit, 48 kHz) has a `<name>.truth.json` beside it.

## Regenerate

```bash
python3 scripts/synth_clips/gen_ticks.py            # all scenarios into assets/synth_clips/generated/
python3 scripts/synth_clips/gen_ticks.py --only g_21600_clean g_21600_snr20
python3 scripts/synth_clips/gen_ticks.py --check g_21600_be_p0.3   # refit rate and BE from the written beats
```

Needs Python 3 with numpy and soundfile. Seeds are fixed, so the output is
reproducible. This README is rewritten by a full run.

## Model

- Three damped-sinusoid bursts per beat: unlock (5 kHz, 0.25 of the drop level),
  impulse (4 kHz, 0.5, mid-lift), drop (3.8 kHz, 1.0, peak 0.08 FS).
- `t_unlock(k) = t0 + k*P + (+e/2 tick, -e/2 tock) + jitter`,
  `P = (3600/bph) / (1 + rate/86400)`, `t_drop = t_unlock + lift(side)`.
  Tick = even `k`, Tock = odd `k`; `k` restarts at 0 in each segment.
- Amplitude truth: `A = L / (2 sin(pi t / T))`, `T = 7200/bph` s, `L = 52` deg.
  The default lift scales with the beat period (9.3 ms at 21,600 bph, about 296 deg).
- Noise is white; SNR is relative to the drop peak. Scenarios without an SNR
  impairment carry 40 dB. Hum is 50/60 Hz plus 2nd and 3rd harmonics (0.5, 0.3).
- Knocks and handling noise are broadband bursts of 5-20 ms.
- Beats whose unlock or drop falls in a dropout carry `"audible": false` (an
  addition to the shared truth format; absent otherwise).
- A drifting rate is integrated beat by beat; the segment's `rate_s_day` is the
  mean over the segment and `rate_start_s_day` / `rate_end_s_day` give the ramp.
- The watch-change scenario has no beats during the handling interval
  (`impairments.handling_s`).

## Scenarios

| File | Length | Size | Truth highlights |
|---|---|---|---|
| `g_18000_clean.wav` | 20 s | 1.92 MB | 18000 bph, rate +6.00 s/d, BE +0.00 ms, lift 11.2/11.2 ms, amp 297/297 deg |
| `g_21600_clean.wav` | 20 s | 1.92 MB | 21600 bph, rate +6.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg |
| `g_28800_clean.wav` | 20 s | 1.92 MB | 28800 bph, rate +6.00 s/d, BE +0.00 ms, lift 7.0/7.0 ms, amp 297/297 deg |
| `g_36000_clean.wav` | 20 s | 1.92 MB | 36000 bph, rate +6.00 s/d, BE +0.00 ms, lift 5.6/5.6 ms, amp 297/297 deg |
| `g_21600_rate_p25.wav` | 20 s | 1.92 MB | 21600 bph, rate +25.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg |
| `g_21600_rate_m30.wav` | 20 s | 1.92 MB | 21600 bph, rate -30.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg |
| `g_21600_be_p0.3.wav` | 20 s | 1.92 MB | 21600 bph, rate +4.00 s/d, BE +0.30 ms, lift 9.3/9.3 ms, amp 297/297 deg |
| `g_21600_be_m0.8.wav` | 20 s | 1.92 MB | 21600 bph, rate +4.00 s/d, BE -0.80 ms, lift 9.3/9.3 ms, amp 297/297 deg |
| `g_21600_lift_unequal.wav` | 20 s | 1.92 MB | 21600 bph, rate +4.00 s/d, BE +0.20 ms, lift 9.0/9.6 ms, amp 307/288 deg |
| `g_21600_snr20.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; SNR 20 dB |
| `g_21600_snr12.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; SNR 12 dB |
| `g_21600_jitter50.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; jitter 50 us RMS |
| `g_21600_jitter300.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; jitter 300 us RMS |
| `g_21600_clip30.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; clip at 0.024 FS |
| `g_21600_hum60.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; hum 60 Hz |
| `g_21600_knocks.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; 8 knocks |
| `g_21600_dropouts.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; 3 dropouts |
| `g_21600_unlock_louder.wav` | 20 s | 1.92 MB | 21600 bph, rate +5.00 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg; unlock louder on tocks |
| `g_change_21600_28800.wav` | 29 s | 2.78 MB | 21600 bph, rate +8.00 s/d, BE +0.20 ms, lift 9.3/9.3 ms, amp 297/297 deg; 28800 bph, rate -5.00 s/d, BE -0.40 ms, lift 7.0/7.0 ms, amp 297/297 deg; handling noise 12-17 s |
| `g_21600_drift.wav` | 30 s | 2.88 MB | 21600 bph, rate +10..+14 s/d, BE +0.00 ms, lift 9.3/9.3 ms, amp 297/297 deg |

Total: 40.2 MB.
