# Plan: mechanical timing engine, informed by opentimegrapher's methods

| | |
|---|---|
| **Status** | Draft for review, 2026-11-23 |
| **Scope** | The mechanical timing chain on core 0: `src/mechanical/tick_detect.cpp`, `beat_fit.cpp`, `beat_analysis.cpp`, and their glue in `src/dsp_graph.cpp` (`analyze()`); the `getBeat` / `beats` protocol fields and the web app's display of them |
| **Reference** | opentimegrapher.org user guide v1.1.0, pages M1 (beats), M2 (rate), M3 (sound card clock), M4 (amplitude), M5 (beat error), M6 (averaging, windows, cuts). Quotes below are short; the pages are the authority on their method |
| **Test bed** | Host harness `tests/dsp_host` (the real firmware chain over a WAV), the ten clips in `assets/synth_clips` with ChronoPulse figures, the board via `TG_SYNTH_CLIP` |
| **Related** | `BLE_TRANSPORT_PLAN.md` §13 (detection pre-filter A10, matched-filter fixes A11): items 1 and 6 here absorb parts of it, see §5 |
| **Workflow** | As before: manager reviews and verifies, sub-agents implement; nobody commits |

---

## 1. Where we stand

The core formulas already agree with opentimegrapher (OTG):

| Quantity | OTG | Ours |
|---|---|---|
| Rate model | `t_k = t0 + P·k ± e/2`, least squares, two refits dropping > 5 robust SD | Same model (`beat_fit.cpp`), one refit dropping > 3.5 σ |
| Rate | `(nominal/P − 1)·86400` | `(1 − P/nominal)·86400` (differs by x² ≈ 0.005 s/d at 20 s/d) |
| Amplitude | `A = L / (2·sin(π·t/T))`, T = two beats | Same (`amplitude_from_lift`) |
| Beat error | half the difference of the Tick→Tock and Tock→Tick intervals | `2|c|` of the fit: the same quantity |

The differences are in **what is timed**, **how landmarks are found** and **how readings are aggregated**:

| | OTG | Ours |
|---|---|---|
| Front end | high-pass > 1.5 kHz, rectify, 0.2 ms smoothing; knocks > 2× tick energy silenced | raw \|x\|, no filter, no smoothing, no knock gate |
| Tick timing | the **drop**, found by sliding a learned median beat template (−20 ms … +8 ms around the drop); two passes; quality score, beats < 0.4 ignored | the **unlock** (first local peak above 25 % of the running peak) |
| Lift time | unlock and drop each timed at **half their own height** on the rising edge; unlock = first steady rise above max(2 % of drop, 4× noise) | unlock = first peak, drop = strongest pulse **peak** (by 0.5 ms mean since today's fix) |
| Amplitude, beat error | **per side** (Tick / Tock) averaged shapes in **2 s windows**, landmarks on the averaged shape, reading = **median of windows**; BE from the unlock, corrected by each side's unlock→drop time, drop-based fallback | per tick; amplitude = median of the last 8 ticks then a time mean; BE = `2|c|` of the unlock-timed fit |
| Rate aggregation | one fit through every good beat in the span; 10 s windows every 5 s for ± and the 5–95 % spread | mean, over `avg_s`, of per-tick rates from overlapping 6 s sliding fits |
| Cuts | nothing fitted or averaged across a cut (> 2 s without a beat, jumps, clock steps); a stretch < 10 s gives no rate; 20 s settling after a position change | grid restart on a long gap or 4 off-grid ticks; history kept across same-rate restarts |
| No watch | beats scattering ≥ 2 % of a beat over ≥ 10 s = no watch | lock = median interval within 4 % of a standard rate; held through noise |

Measured symptoms that point at these differences (clips, 2026-11-23): fit jitter **140–390 µs RMS** on clean recordings; amplitude **+6 to +16°** above ChronoPulse's onset figure on most clips; beat error **lower than ChronoPulse** on several (0.02 vs 0.3 ms on `m8215_incase`); lock held through 5 s of watch-change noise.

## 2. Phase 0: measurement before change (prerequisite for every item)

Nothing below changes firmware until its effect is measured on the same data. Phase 0 builds the yardsticks once.

**0a. Ground truth we control: a parametric tick generator** (`scripts/synth_clips/gen_ticks.py`). The ChronoPulse figures are one instrument's opinion, not truth. The generator writes WAVs of ticks built from three bursts (unlock, impulse, drop: damped sinusoids at 3–8 kHz) with **exactly known** rate, beat error (signed), lift time per side (so amplitude for a given λ), jitter, noise level, clipping level, knocks, unlock-louder-than-drop cases, dropouts and a watch change. Ground truth is written to a JSON beside each WAV. About 20 scenarios; they go into `assets/synth_clips/generated/` and the harness.

**0b. Harness metrics** (`tests/dsp_host`). Add `--beats` (one line per beat: landmark times, per-beat lift, side, status) and a `--truth file.json` mode that prints errors against truth: rate error (s/d), BE error (ms), amplitude error (°) per side, fit jitter (µs RMS), lock fraction, amplitude-valid fraction, time to first valid reading, and a CPU proxy (host ns per block, ratio to baseline). A script runs every clip and generated scenario and prints one comparison table, so every item below reports "before / after" from the same run.

**0c. Python reference of the OTG method** (`scripts/synth_clips/reference_otg.py`), offline, not real-time: envelope, template drop timing, half-height landmarks, per-side 2 s windows, unlock-corrected BE. Running it on the same data tells us, before writing C++, how much each change is worth and whether it matches ChronoPulse better. Items 1–3 each start by switching the relevant step of the reference on and off.

**Acceptance of Phase 0:** the table runs in under a minute on the host; the reference reproduces ChronoPulse's amplitude within ±10° and rate within ±0.3 s/d on at least 7 of 9 clips (if it does not, we have learnt that OTG's method alone does not explain ChronoPulse, which narrows the search).

## 3. Items, most impactful first

Impact = predicted effect on the readings a user sees (rate, amplitude, beat error and their stability), weighed by how often the situation arises. Each item lists the predicted gain and how it is checked; the ranking is revisited after Phase 0's reference runs.

### Item 1: time the drop with a beat template (and the envelope front end it needs)

**Why first.** Every reading depends on the per-beat timestamp. We time the unlock, the quietest and least repeatable landmark, from raw samples: the fits show 140–390 µs RMS jitter. Rate uncertainty over a span scales with jitter; at 300 µs and 21,600 bph a 10 s span gives roughly ±0.5 s/d, so readings wander and settle slowly. OTG times the drop by matching the whole beat shape, which also cannot jump between unlock and drop when their loudness changes ("matching the whole shape avoids jumping to the unlock on odd beats where it is louder than the drop"). **Predicted:** jitter down 2–5×, faster settling, tighter paper strip, and the fit's (drop-based) beat error becomes usable as a fallback for item 3.

**Design: a two-stage detector.**

- **Stage A (existing, per sample, cheap):** `tick_detect_block` keeps finding tick windows with its threshold and noise floor. It already runs on every sample; it stays the "is there a tick, roughly where" gate and keeps rate locking working while the template learns.
- **Envelope front end (new, per sample):** a 4th-order Butterworth high-pass at 1.5 kHz (two CMSIS biquad stages, ~10 MACs/sample), rectify, 0.2 ms boxcar (running sum, the same structure as today's pulse-strength window), then decimate by 2 to 24 kHz into an **int16 ring of 40 ms** (960 samples, 1.9 KB). The ring always holds the last 40 ms, so a closed event has its pre-roll. The envelope also feeds the detector threshold (a filtered, smoothed signal makes thresholds steadier than raw |x|), switchable for A/B.
- **Stage B (new, per event, at close):** copy the window around the event's strongest region from the ring and:
  1. While no template exists: take the drop as the envelope peak (by 0.5 ms strength, as today) and record the shape, aligned on that peak, into the **template accumulator** (mean of the first 16 good beats, per side, then an exponential update at 1/32 per beat). Span −20 ms … +8 ms = 672 samples at 24 kHz, int16, per side: 2.7 KB for both.
  2. With a template: cross-correlate the template against the window over ±2 ms of lag (±48 lags × 672 = 32 k MACs per beat; at 10 beats/s ≈ 0.3 M MACs/s, under 2 % of core 0), take the best lag, refine it with a parabolic fit to the three best correlation values (sub-sample), and report a **quality** = normalised correlation (0..1). Beats below 0.4 are not used for timing (OTG's figure; tuned on the clips).
  3. The beat's timestamp for the fit becomes the drop time (template lag + the template's own drop half-height crossing, so the timestamp is a physical landmark, not an arbitrary alignment point).
- **beat_analysis:** a new parameter `timingLandmark` (`unlock` | `drop`), default `unlock` until item 1 passes, then `drop`. Lock still comes from the median interval (unchanged). The fit sees drop times.
- **Matched filter (§13.4):** becomes redundant: the template *is* a learned beat shape, used better (timing, not just a veto). Retire `matchedFilter` once item 1 ships, rather than doing A11.

**Costs.** RAM ≈ 1.9 KB (ring) + 2.7 KB (templates) + 2 KB (correlation scratch, could reuse the event wave buffer) ≈ **6.6 KB**: more than `pico2w_pcm1808`'s 5.8 KB headroom under BLD-02. Options, decided in review: raise BLD-02 (it is our own 50 % budget, not a hardware limit; 512 KB SRAM), or pay for it by retiring the matched filter (~0.6 KB), the JSON `beats` encoder's 32-beat scratch (1.3 KB, the user's "first to go" rule), and the 64-sample event wave (0.25 KB, replaced by the ring). CPU: per-sample filter + envelope ≈ 20 % more than today's analysis cost; per-beat correlation as above. Both measured with `getTiming` on the board.

**Checks.** Generated scenarios: timestamp error vs truth < 20 µs RMS at 30 dB SNR; no unlock/drop jumping on the "unlock louder" scenario. Clips: jitter down ≥ 2× on 7/9; rate vs the designed loop rate unchanged (±0.1 s/d); lock and amplitude-valid fractions not worse. Board: DSP block max < 2.5 ms.

### Item 2: half-height landmarks and per-side 2 s shape windows for amplitude

**Why.** Amplitude is very sensitive to the lift time (OTG: 1 ms ≈ 37° at 28,800 bph). We time lift between two **peaks**; OTG between two **half-height crossings** of the rising edges. The peak of a burst sits a fraction of a millisecond after its onset, and that offset differs between the small unlock and the big drop, so peak-to-peak lift is biased. That is the likeliest cause of our +6 to +16° over ChronoPulse's onset figure. Averaging each side's beats into one shape before finding landmarks raises the SNR by √N (≈ 4× for 2 s at 21,600 bph), which matters for quiet pickups where single-beat unlocks are lost in noise. It also gives **Tick and Tock amplitudes**, which users read as a diagnostic. **Predicted:** amplitude bias vs a half-height reference drops to < 5°, amplitude-valid fraction on noisy recordings up, new per-side figures.

**Design.**

- **Shape windows (new, per side):** each beat that passed item 1's quality gate is added, aligned on its drop, into its side's accumulator (int32 sums over −20 … +8 ms at 24 kHz: 672 × 4 B × 2 sides = 5.4 KB; or reuse the template span and int16 means at 2.7 KB if RAM is short). Every 2 s of beats (OTG: laid end to end, not overlapping) the window closes. OTG keeps "the middle half of beats at each moment" (a per-sample trimmed mean), which needs every beat stored: too much RAM here, so we use a plain mean of quality-gated beats and measure whether the trim matters with the Python reference first.
- **Landmarks on the averaged shape:** drop = rising-edge half-height crossing, found by walking back from the drop peak (≤ 2 ms); unlock = first steady rise above max(2 % of the drop height, 4× the background noise of the shape's pre-roll), then its own half-height crossing. Linear interpolation between envelope samples gives sub-sample times.
- **Per-window result:** lift time per side → amplitude per side; window amplitude = mean of the two sides (OTG) when both have ≥ 3 beats; a side short of beats keeps its own figure.
- **Reading:** median of window amplitudes over the averaging span (OTG M6), replacing today's "median of last 8 ticks, then time mean". Per-beat amplitude in the `beats` stream stays (single-beat landmarks, labelled as such).
- **Protocol:** `getBeat` gains `amplitudeTickDeg`, `amplitudeTockDeg`, `amplitudeWindows` (used / total). Additive; d.ts, HID doc, web Timegrapher view.

**Checks.** Generated: per-side amplitude within ±5° of truth at 25 dB SNR and with clipping. Clips: compare against ChronoPulse onset; amplitude-valid windows on all 9 clips.

### Item 3: beat error from the unlock, corrected by each side's lift (drop-based fallback)

**Why.** Beat error is about the *escapement's* symmetry, which the unlock reflects; the drop's timing also carries each side's lift time, so a drop-only BE mixes in lift asymmetry. OTG computes BE from the unlock on the per-side shapes, corrected: `BE = drop BE − Tick lift + Tock lift`, median of 2 s windows, and falls back to the drop-based fit when windows disagree in sign beyond their spread. Our BE reads low against ChronoPulse on several clips; the method difference is the first suspect. **Predicted:** BE agreement with ChronoPulse within ±0.1 ms on the clips; a signed BE (which side is late), and a stable Tick/Tock labelling rule.

**Design.**

- Inputs come for free from items 1 and 2: the drop-based `c` of the fit (BE_drop = 2c, signed) and each window's per-side unlock→drop times.
- Window BE (signed) = BE_drop − lift_tick + lift_tock; reading = median of windows.
- **Fallback:** if fewer than 3 windows, or the windows' sign disagrees more than their spread allows, report the drop-based figure and say so (`beatErrorSource: "unlock" | "drop"`).
- **Labelling:** OTG names the side that comes late over the first seconds "Tick" and starts positive. We need a rule that is stable across grid restarts within one watch; adopt OTG's and keep it until the nominal rate changes.
- **BE angle** (item 8) falls out here.
- **Protocol:** `beatErrorSignedMs`, `beatErrorSource`; `beatErrorMs` stays the magnitude.

**Checks.** Generated scenarios with known signed BE and unequal lift per side: |error| < 0.05 ms. Clips: vs ChronoPulse.

### Item 4: cuts and "no watch"

**Why.** Today lock rides through noise (the watch-change clip: lock held through 5 s of handling, the average drifted ~3 s/d), and a fit or average can span a pause. OTG never fits or averages across a cut, gives no rate from a stretch under 10 s, and calls it "no watch" when beats scatter by ≥ 2 % of a beat over ≥ 10 s. **Predicted:** no bogus readings across watch changes, knocks and pauses; a clear "no watch" state instead of stale numbers. High value for real use (every watch change), low effect on steady readings, hence fourth.

**Design.**

- **Cut detector** in `beat_analysis`: (a) more than 2 s without an accepted beat; (b) a timing jump: k consecutive beats off the current grid by more than a threshold (we already count `BEAT_FIT_MAX_REJECTS`; make the restart a cut); (c) the nominal rate changes (done today); (d) the timebase correction steps (`timebase_ppm` changes: the beat record notes this already, "hosts should treat that as a new grid"); (e) capture stop/start (stream reset, done).
- At a cut: close the stretch; nothing carries over into the next stretch's fit, windows or averages. Readings over a span combine stretches weighted by beats used (rate, drop BE) or as a median of windows (amplitude, unlock BE), per OTG M6.
- **No-watch rule:** over the last 10 s, if the scatter of accepted beat times about the fit is ≥ 2 % of the half-period (e.g. ≥ 3.3 ms at 21,600 bph), or fewer than half the expected beats are good, drop lock (`locked = false`, new `state: "noWatch"`).
- **Short stretch:** no rate from a stretch spanning < 10 s (today: as soon as the fit has 8 points and spans ¾ of its 6 s window); a first reading after ~10 s matches OTG's "a few seconds of beats give a rate several s/d out".
- **Settling** after a cut (OTG: 20 s after a position change): a parameter `settleS`, default 0 for live use (the user sees "settling"), so the averaging span starts after it.

**Checks.** Switch clip: lock drops within 2 s of the changeover noise, no rate shown until 10 s of the new watch, no average drift; generated scenarios with knocks and pauses: no reading contains beats from both sides of a cut.

### Item 5: rate over the span as one fit (sufficient statistics), with ± and spread

**Why.** Our long average is a mean of per-tick rates from overlapping 6 s fits: correlated samples, an implicit weighting that over-weights the middle of the span, and no honest ±. OTG fits one line through every good beat in the span and derives ± and a 5–95 % spread from 10 s windows every 5 s. **Predicted:** small accuracy gain (the estimates are similar on steady watches), a real ± and spread for the display, exactness when the span crosses a slow drift.

**Design: sufficient statistics per 1 s block.** The fit `t = a + P·k + c·s` (s = ±1 by parity) is linear least squares, so the exact fit over any set of beats is determined by a handful of sums: `n, Σk, Σk², Σs, Σks, Σt', Σkt', Σst'` with `t'` the time relative to the stretch's origin (doubles). Keep one such record per 1 s block (8 doubles + 1 int ≈ 72 B; 60 blocks = 4.3 KB for a 60 s span, or 1 KB at 5 s blocks). Then:

- **Span rate** = solve the summed blocks inside the span and the current stretch: exact, no point storage, outlier rejection done per beat before it enters a block (against the current 6 s fit, as today).
- **10 s windows every 5 s** = two adjacent 5 s groups of blocks: rate per window, ± from the windows (OTG notes neighbouring windows are correlated; with 50 % overlap use the non-overlapping subset for the ±, all windows for the chart).
- **Spread** = 5th–95th percentile of window rates.
- Drop-based BE over the span comes from the same solve (`c`).
- The existing 6 s sliding fit stays for per-beat residuals, outlier rejection and the paper strip.

**Protocol:** `getBeat.avgLong` keeps its fields (values now from the span fit); adds `rateUncertaintySDay`, `rateP5SDay`, `rateP95SDay`, `beatsUsed`. **Checks:** generated scenarios with known rate and a slow drift: span rate error < 0.05 s/d; ± covers truth ~68 % of the time over many seeds.

### Item 6: detection front end and knock gating

**Why.** OTG high-passes above 1.5 kHz and silences knocks (bursts with > 2× a typical tick's energy, shorter than 0.1 s). On real pickups, handling noise and bench knocks are frequent; on our clean clips the effect is small, hence sixth despite being important in the field. **Overlap:** item 1 brings the 1.5 kHz high-pass and envelope; this item adds the **knock gate** in stage B (reject a beat whose envelope energy exceeds 2× the running median of accepted beats) and makes stage A's threshold run on the envelope. `BLE_TRANSPORT_PLAN.md` §13's `detectFilter` (notch + linear-phase FIR band-pass, delay-compensated) becomes the configurable front end ahead of the envelope: keep its design, drop its separate matched-filter work (A11, see item 1). **Checks:** generated knock and hum scenarios: no knock timed as a beat; rate unaffected; hum scenarios with the notch.

### Item 7: averaging-window semantics and the display (small)

OTG's chips ("6 s of beats", "540 of 600 beats", "amplitude read from 4 of 15 windows") make the reading's support visible. Add `beatsUsed`, `beatsExpected`, `windowsUsed` to `getBeat` (most come from items 2 and 5) and show them in the web app. Low effort once items 2 and 5 exist.

### Item 8: beat-error angle and small exactness fixes (trivial)

- `BE angle (°) = amplitude (°) × π × bph / 7200 × BE (s)` (OTG M5): comparable across amplitudes. One field in `getBeat`, one value in the app.
- Rate formula: use `(nominal/P − 1)·86400` to match OTG and most instruments exactly (≈ 0.005 s/d at ±20 s/d today).
- Timebase: our correction scales the sample clock exactly (`fs_true = fs·(1 + ppm·10⁻⁶)`), equivalent to OTG M3's exact form; add a harness case with a ±50 ppm source clock to lock that in. OTG's "USB lines" trick (timing the 1 kHz USB frame tones) does not apply: the stand is its own sound card and has calibration and GPS PPS.

## 4. Order of work and dependencies

```
Phase 0 (0a generator, 0b harness metrics, 0c OTG reference) ──► review: re-rank items 1–3 from the reference's numbers
   │
   ├─► Item 1 (envelope + template drop timing) ──► Item 2 (half-height, per-side windows) ──► Item 3 (unlock BE)
   │                                   └──────────► Item 6 (knock gate; §13 detectFilter as front end)
   ├─► Item 4 (cuts, no-watch)  [independent of 1–3; can run in parallel]
   └─► Item 5 (span fit by sufficient statistics) ──► Item 7 (support figures) ; Item 8 any time
```

| Task | Model | Depends on | Deliverable |
|---|---|---|---|
| T0a tick generator + scenarios | sonnet | — | `gen_ticks.py`, ~20 WAV + truth JSON |
| T0b harness metrics + comparison table | sonnet | T0a | `--beats`, `--truth`, `scripts/synth_clips/compare.py` |
| T0c OTG reference (offline) | sonnet | T0a | `reference_otg.py`, table vs ChronoPulse and truth |
| T1 envelope front end + template drop timing | opus-reviewed sonnet | T0b | `tick_shape.{h,cpp}` (new), `beat_analysis` landmark switch, harness rows |
| T2 half-height landmarks + per-side windows | sonnet | T1 | amplitude per side, `getBeat` fields |
| T3 unlock-corrected BE + labelling | sonnet | T2 | signed BE, source, fallback |
| T4 cuts + no-watch | sonnet | T0b | cut detector, `state`, short-stretch rule |
| T5 span fit by sufficient statistics | sonnet | T0b, T4 | block statistics, ±, spread |
| T6 knock gate + detectFilter as front end | sonnet | T1 | (replaces BLE plan A10/A11 scope) |
| T7 support figures, T8 BE angle + formula | haiku | T2, T5 | fields, docs, web display |
| W7 web app | sonnet | T2, T3, T5 | per-side amplitude, signed BE + source, ±/spread, support chips, BE angle, "no watch" |

Every firmware task: host rows in `tests/run_all.sh` that fail before and pass after (the pattern used for today's ST3600 and switch fixes), the comparison table before/after in the task report, `getTiming` on the board for CPU, RAM per env against BLD-02.

## 5. Interaction with existing plans and fields

- **BLE plan §13** (detection pre-filter A10, matched-filter fixes A11, synthetic impairments A12, timing stats A13): A10's `detectFilter` becomes item 6's front end; **A11 is superseded** by item 1's template (retire `matchedFilter`); A12 (synthetic impairments: hum, noise, clicks) is still useful and partly covered by T0a; A13 unchanged.
- **Protocol:** all additions are new fields; `v` / `protocolVersion` stay 1. `timingLandmark` and the shape-window method are parameters, so the old behaviour stays selectable for A/B on hardware until the new one is accepted.
- **Packed beats** (`beatsPacked`): per-beat records keep their layout; the timestamp they carry follows `timingLandmark` (documented). A `flags` bit for "drop-timed" can be added in format 2 if hosts need it.
- **RAM:** items 1 + 2 + 5 add ≈ 6.6 + 2.7…5.4 + 1…4.3 KB. Decide in review whether to raise BLD-02 or retire the matched filter and the JSON `beats` encoder first.

## 6. Decisions (2026-11-23)

- **RAM:** BLD-02 raised to 60 % (314,572 B on the W builds), and the matched filter is retired (superseded by item 1's template). The JSON `beats` stream stays.
- **Default timing:** hybrid. Drop mode starts with unlock-timed readings while the templates learn, then hands over to drop timing on the fly (the fit's points are shifted onto the drop time base; not a cut).

## 7. Open questions for the user (original)

1. **Default landmark:** switch rate timing to the drop once item 1 passes, or keep the unlock as default and offer the drop as an option?
2. **RAM budget:** raise BLD-02 (our 50 % rule) or pay for the new buffers by retiring the matched filter and the JSON `beats` encoder?
3. **Settling time** after a watch change (OTG: 20 s, not read): default on for standalone use, off for USB-mic use?
4. **More recordings:** the results file lists three "On_Off" recordings (`21600BPH_NH35_On_Off45_on60.wav` etc.) that were not in the folder. They exercise cuts and re-locks (item 4); can they be added?

## 8. USB Audio mode: timing off (user request, 2026-11-23)

"When in USB Audio mode, turn off the beat analysis / timing engine(s). OLED should note that it is in USB Audio mode in the main section of the screen. Keep the status bar as-is, detect the beat-rate, but that is it."

- **U1 (OLED):** in USB microphone mode the header is unchanged (nominal bph or status text, mode tag, Bluetooth and GPS badges; quartz header in quartz modes); everything below the header rule becomes a "USB Audio" section. Standalone is unchanged. `docs/DISPLAY.md` and the renders follow.
- **U2 (engine, after T2/T3):** in USB microphone mode only tick detection and the beat-rate lock run (interval median → nominal bph, `locked`, `vphNearest`). Off: the rate/beat-error fit, rolling means, stage B (envelope front end, templates, drop timing), amplitude and beat-error windows, the no-watch rule, and the quartz engine. The playback path (filters for the USB stream) is untouched. Switching to standalone restarts the engine cleanly (a cut). `getBeat` reports `analysis: "off"` (with `locked` / `vphNearest` still live) so hosts can say why there is no rate; `getQuartz` likewise. The web app shows "USB Audio mode: timing runs on the host" in place of the readings. Host rows: in USB mode, no rate/BE/amplitude, lock and bph present; CPU per block lower than standalone (board `getTiming`).
