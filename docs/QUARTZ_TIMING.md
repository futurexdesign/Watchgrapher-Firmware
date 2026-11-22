# Quartz-watch timing

The stand can time quartz watches as well as mechanical ones. The quartz engine
(`include/quartz/quartz_engine.h`) runs one measurement at a time, chosen by the
**quartz mode**:

| Mode | Watch | Channel / pickup | What is measured | Handoff |
|---|---|---|---|---|
| `mechanical` | Mechanical (default at boot) | — | Quartz timing off; the mechanical beat path runs in every mode | — |
| `motor` | Analog quartz (stepper motor) | LEFT, inductive coil (JP1) | Rate with and without inhibition from the drive-pulse leading edges | §4, §5 |
| `lcd` | LCD digital | RIGHT, capacitive electrode (JP2 open) | Oscillator rate from the LCD drive harmonics | §6 |
| `fork` | Tuning fork (Accutron and similar) | LEFT | Rate against the nominal fork frequency | §7 |

The same engine is reachable from three front ends: the CDC serial CLI, the
WebHID control plane and the OLED display. All three read the same
`control_api_get_quartz()` snapshot (`include/dsp_quartz_types.h`).

Sections: [1. Concepts](#1-concepts) · [2. CLI](#2-cli) ·
[3. HID](#3-hid) · [4. Display](#4-display)

## 1. Concepts

These apply to every front end.

**Requirements.** Quartz timing exists only on the PCM1808 builds
(`pico2_pcm1808`, `kiwi_pcm1808`). It measures only once the capture rate is
verified and is exactly 48 kHz. Until then the state is `waitTimebase` or
`unsupportedRate`. On other builds the mode stays `mechanical` and every setter
is refused.

**Rates.** These are given in seconds per day, positive when the watch gains.
They are corrected by the sample-clock calibration (`timebase`, see
`docs/TIMEBASE_CALIBRATION.md`). Without a calibration they are flagged
**uncalibrated**: an uncorrected board can be off by ±1.7 s/d at ±20 ppm. Every
rate is also available against the raw board clock (`uncal`).

**Running without a host.** Quartz timing needs capture to run, which it does while the USB audio stream is open, after `run start`, or on its own in standalone device mode (`device mode standalone`; see `docs/AS_BUILT_ARCHITECTURE.md` §4.3). A standalone device boots in `mechanical`, because the quartz mode is not saved, so select the quartz mode over the CLI or HID after boot. With nothing running the OLED status line reads `no host stream` (USB microphone mode) or `stopped` (standalone).

A flash write (`timebase save`, a device-mode or `logdelta` change, a calibration-log write) pauses capture and restarts it as a fresh stream, which starts a new measurement. The automatic GPS-PPS log write therefore waits while a measurement is acquiring or measuring (see `docs/TIMEBASE_CALIBRATION.md`); a manual save does not wait.

**States.** `idle` (mechanical) → `waitTimebase` / `unsupportedRate` (gated) →
`acquiring` (running, no rate yet) → `measuring` (a rate exists and is being
refined) → `done` (motor gate reached, result frozen). Any of the following
starts a new measurement: a mode change, a change to a parameter that the active
mode uses, `restart`, a stream restart, a change of source rate or timebase, or
captured sample loss.

**Inhibition (motor only, handoff §3.2 / §5).** Most quartz watches are trimmed
digitally. Every *P* seconds (typically 10 or 60 s, up to 960 s) the IC deletes
*n* pulses at the 8192 Hz divider stage. Each deletion delays the next step by
n × 122.07 µs. The motor result therefore carries two rates:

- **rate without inhibition**: the raw oscillator, measured within a segment.
- **rate with inhibition**: the mean over whole inhibition cycles. This is what
  the wearer sees and is the headline, but it exists only after **two**
  inhibition events.

| Status (`inhibition=`) | Shown as | Meaning |
|---|---|---|
| `none` | no rate yet: waiting for 3 pulses | Fewer than 3 pulses |
| `may_include_inhibition` | raw rate, may include inhibition: use a longer gate | No jump yet, and the span is short |
| `need_one_more_cycle` | raw rate, inhibition seen once: need one more cycle | One jump |
| `correction_pending` | raw rate, 2 inhibition events: corrected at the next pulse | Two jumps; waiting for the next same-polarity pulse (transient) |
| `corrected` | corrected for inhibition | Headline = rate with inhibition |
| `no_jump_long_gate` | no inhibition detected | No jump > 20 µs in a gate of at least 120 s (twice the 60 s longest expected period) |

A jump is a step interval more than 20 µs too long. The jump divided by
122.07 µs gives *n* at 8192 Hz, a figure watchmakers use.

**Motor gate.** The gate is either `auto` (the default; also written `0`) or a
number of seconds from 3 to 960. Reaching it freezes the result (`done`) until a
restart.

- `auto` freezes as soon as the corrected rate exists, that is, after 2
  inhibition events. A watch without inhibition never freezes. It keeps
  refining, and once it has covered 120 s with no jump it reports "no
  inhibition detected".
- A timed gate shorter than two inhibition cycles can only report the raw rate.
  For example, 10 s on a 60 s-inhibition watch reads "may include inhibition".
  This is expected, not a fault (handoff §8).

**LCD versus motor.** The LCD is driven from the divider *before* the trimming,
so the LCD channel cannot see inhibition. On the same watch, the LCD rate equals
the motor rate *without* inhibition. LCD and motor headline rates can therefore
legitimately differ (handoff §8).

**Jumpers (handoff §2, §8).**

| Jumper | Channel | Default | Guidance |
|---|---|---|---|
| JP1 | LEFT (inductive coil): motor, fork | Bridged (×11) | Suits 20–1000 µV pulses but clips at about 1.4 mV EMF. Open it (×1) when the status says "signal clipping: open JP1". |
| JP2 | RIGHT (capacitive electrode): LCD | Open (×1) | Keep it open for LCD. When bridged, it clips above about 0.33 pF of LCD coupling. |

**Hints.**

- "weak signal: reposition the watch" appears when the median pulse SNR is below
  16 (about 30 µV EMF). The rate still converges, but per-pulse jitter exceeds
  2 µs.
- Extra bursts in a pulse group are detection or correction pulses. They are
  logged as a fault indication and never timed.

## 2. CLI

The `quartz` command (alias `qz`) on the USB CDC serial console. The command
follows the CLI conventions: replies start with `CLI: `, values use `key=value`,
and errors start with `CLI: ERR`. `help quartz`, `quartz help` and `quartz ?`
print the overview. `quartz help <sub>` and `quartz <sub> help` print one
subcommand's reference. The top-level `status` command ends with the quartz
headline line.

| Command | Purpose |
|---|---|
| `quartz` / `quartz status` | Full readout, headline first |
| `quartz mode [mechanical\|motor\|lcd\|fork]` | Show or select the mode |
| `quartz gate [0\|auto\|<3..960>]` | Show or set the motor gate (s) |
| `quartz fork [<Hz>]` | Show or set the tuning-fork nominal frequency |
| `quartz ts [threshold\|xcorr]` | Show or set the motor timestamp method (alias `tsmethod`; also `0`/`1`) |
| `quartz ksigma [<4..20>]` | Show or set the motor detection threshold (alias `k_sigma`) |
| `quartz restart` | Start a new measurement (alias `reset`) |
| `quartz pulses [1..16]` | List recent motor pulses (default 8) |
| `quartz help [<sub>\|inhibition\|jumpers]` | Print help; `inhibition` prints the §5 table, `jumpers` the JP1/JP2 guide |

The engine checks every parameter value (`control_api_set_quartz_param_ex`).
A refused set prints the engine's reason (`outOfRange`, `unknownName`) and then
the accepted form, which is built from the engine constants:
`CLI: ERR <param> <reason> - usage: ...`. A token that is not a number (or not
a method name) prints `CLI: ERR bad <param> value '<token>' - usage: ...`. On
a build without PCM1808, every parameter command prints
`CLI: ERR quartz timing needs the PCM1808 source (this build: ADC)`.

Parameters and the mode are not saved (only the device mode and the timebase are persisted): the board always boots in `mechanical`
with gate `auto`, fork 360 Hz, `threshold` timestamps and kSigma 8. A set that
changes a parameter of the active mode restarts the measurement, and the reply
says so.

The numbers in the examples below are illustrative. The line layouts match the
firmware.

### 2.1 `quartz status`

The first line is the headline: mode, state, the headline rate, the signal time
`t`, the motor gate, and what the rate means. The rate is tagged `UNCAL` when no
calibration exists, and `done` appends the reason the result froze. The
following lines give the gating inputs, the parameters, the calibration and the
active mode's detail. `uncal=` is the same rate against the raw board clock.

Motor, gate `auto`, after two inhibition events (frozen):

```
> qz
CLI: quartz motor done rate=-0.412 s/d t=31.0s gate=auto - corrected for inhibition; two inhibition events seen
CLI: quartz mode=motor state=done src_hz=48000 timebase=ok elapsed_s=31.0 gate=auto loss_restarts=0 reason="two inhibition events seen"
CLI: quartz params ts=threshold ksigma=8.0 fork_hz=360
CLI: quartz cal_ppm=-23.400 cal_source=ntp calibrated=yes
CLI: quartz motor pulses=31 fit=31 step_s=1 span_s=30.0
CLI: quartz motor rate_with=-0.412 (uncal=1.610) rate_without=2.752 (uncal=4.774) s/d
CLI: quartz motor inhibition=corrected "corrected for inhibition"
CLI: quartz motor events=2 cycles=1 period_s=10.0
CLI: quartz motor jump_us=366.2 n=3 jump_mean_us=366.2 n_mean=3
CLI: quartz motor residual_rms_us=0.35 polarity_offset_us=1.20
CLI: quartz motor noise_sigma=9905 threshold=79240 peak_snr=51.5
CLI: quartz motor extras=0 clipped=no clipped_pulses=0
```

Motor, 10 s gate on a watch with 60 s inhibition, no calibration, the watch
too close to the coil (clipping, one correction pulse):

```
> quartz status
CLI: quartz motor done rate=2.751 s/d UNCAL t=10.0s gate=10s - raw rate, may include inhibition: use a longer gate; gate reached
CLI: quartz mode=motor state=done src_hz=48000 timebase=ok elapsed_s=10.0 gate=10s loss_restarts=0 reason="gate reached"
CLI: quartz params ts=threshold ksigma=8.0 fork_hz=360
CLI: quartz cal_ppm=0.000 cal_source=none calibrated=no
CLI: quartz UNCALIBRATED: rates use the raw board clock (see 'help timebase')
CLI: quartz motor pulses=10 fit=10 step_s=1 span_s=9.0
CLI: quartz motor rate_with=2.751 (uncal=2.751) rate_without=2.751 (uncal=2.751) s/d
CLI: quartz motor inhibition=may_include_inhibition "raw rate, may include inhibition: use a longer gate"
CLI: quartz motor events=0 cycles=0 period_s=n/a
CLI: quartz motor residual_rms_us=0.02 polarity_offset_us=0.85
CLI: quartz motor noise_sigma=9890 threshold=79120 peak_snr=848.1
CLI: quartz motor extras=1 clipped=yes clipped_pulses=10
CLI: quartz fault: 1 detection/correction pulse(s) (log as fault indication)
CLI: quartz hint: signal clipping: open JP1
```

With a weak pickup (median `peak_snr` below 16) the last line is instead
`CLI: quartz hint: weak signal: reposition the watch`.

LCD:

```
> qz
CLI: quartz lcd measuring rate=0.873 s/d t=12.5s - oscillator rate, f0=64Hz
CLI: quartz mode=lcd state=measuring src_hz=48000 timebase=ok elapsed_s=12.5 gate=auto loss_restarts=0 reason=""
CLI: quartz params ts=threshold ksigma=8.0 fork_hz=360
CLI: quartz cal_ppm=-23.400 cal_source=ntp calibrated=yes
CLI: quartz lcd status=tracking f0_hz=64 harmonics=13,11,15,9 blocks=21 retries=0
CLI: quartz lcd rate=0.873 (uncal=2.895) s/d clipped=no
CLI: quartz note: the LCD cannot see inhibition: this is the oscillator rate and may differ from motor mode
```

Fork:

```
> qz
CLI: quartz fork measuring rate=-1.204 s/d t=9.0s - f_nom=360Hz
...
CLI: quartz fork f_nom_hz=360 blocks=36 rate=-1.204 (uncal=0.818) s/d clipped=no
```

Gated states and mechanical mode print the headline, the gating line, the
parameters, the calibration and a tip:

```
> qz
CLI: quartz motor waitTimebase - capture rate not verified
CLI: quartz mode=motor state=waitTimebase src_hz=48000 timebase=unverified elapsed_s=0.0 gate=auto loss_restarts=0 reason="capture rate not verified"
CLI: quartz params ts=threshold ksigma=8.0 fork_hz=360
CLI: quartz cal_ppm=0.000 cal_source=none calibrated=no
CLI: quartz UNCALIBRATED: rates use the raw board clock (see 'help timebase')
CLI: tip: waiting for the capture rate to be verified (see status)
```

On a build without PCM1808 (ADC or SYNTHETIC), only the first two lines are
printed:

```
> qz
CLI: quartz mechanical idle - quartz timing needs the PCM1808 source
CLI: quartz mode=mechanical state=idle src_hz=0 timebase=unverified elapsed_s=0.0 gate=auto loss_restarts=0 reason="quartz timing needs the PCM1808 source"
```

The top-level `status` command ends with the same headline:

```
> status
CLI: mounted=true streaming=true state=active source=PCM1808 dsp_bypass=true uptime_ms=812345
CLI: pcm1808 rate=ok measured_hz=48000.012 mclk_ratio=256 upsample=2 dsp_hz=48000 timebase=ok
CLI: device mode=usbMic running=yes run_requested=no usb_streaming=yes stream=active saved=no
CLI: quartz motor measuring rate=-0.398 s/d t=42.0s gate=60s - corrected for inhibition
```

### 2.2 `quartz mode`

```
> quartz mode
CLI: quartz mode=mechanical
> quartz mode motor
CLI: quartz mode=motor (new measurement)
CLI: tip: motor reads LEFT (inductive coil). JP1 bridged (x11) by default; open it on 'signal clipping'
> qz mode lcd
CLI: quartz mode=lcd (new measurement)
CLI: tip: lcd reads RIGHT (capacitive electrode), JP2 open. Inhibition is invisible on the LCD
> qz mode fork
CLI: quartz mode=fork (new measurement)
CLI: tip: fork reads LEFT at f_nom=360 Hz (quartz fork <Hz>); allow at least 8 s
> qz mode fork
CLI: quartz mode=fork (unchanged; 'quartz restart' starts over)
> qz mode quartz
CLI: ERR unknown mode 'quartz' (mechanical|motor|lcd|fork)
```

On a build without PCM1808, `quartz mode mechanical` succeeds and any other mode
is refused:

```
CLI: ERR quartz timing needs the PCM1808 source (this build: ADC)
```

### 2.3 `quartz gate`

```
> quartz gate
CLI: quartz gate=auto (until 2 inhibition events)
> quartz gate 60
CLI: quartz gate=60s (measurement restarted)
> quartz gate auto
CLI: quartz gate=auto (until 2 inhibition events) (measurement restarted)
> quartz gate 2
CLI: ERR gate outOfRange - usage: quartz gate [0|auto|<3..960 s>]
> quartz gate soon
CLI: ERR bad gate value 'soon' - usage: quartz gate [0|auto|<3..960 s>]
```

`(measurement restarted)` appears only when the value changed and the mode is
`motor`. In any other mode the gate is stored for later.

### 2.4 `quartz fork`

```
> quartz fork
CLI: quartz fork_hz=360
> quartz fork 341.33
CLI: quartz fork_hz=341.33
> quartz fork 400
CLI: ERR forkHz outOfRange - usage: quartz fork <300|341.33|360|480|720> (Hz)
```

The allowed list comes from `quartz::kForkFrequencies`. `341.33` selects exactly
1024/3 Hz; the engine matches within 0.01 Hz.

### 2.5 `quartz ts` and `quartz ksigma`

```
> quartz ts
CLI: quartz ts=threshold
> quartz ts xcorr
CLI: quartz ts=xcorr (measurement restarted)
> quartz ksigma
CLI: quartz ksigma=8.0
> quartz ksigma 6
CLI: quartz ksigma=6.0 (measurement restarted)
> quartz ksigma 30
CLI: ERR kSigma outOfRange - usage: quartz ksigma <4..20>
> quartz ts fast
CLI: ERR bad tsMethod value 'fast' - usage: quartz ts [threshold|xcorr]
```

- `threshold` (the default) times the interpolated 50 % crossing of the leading
  spike.
- `xcorr` cross-correlates each pulse with a template that is fixed after the
  first 4 pulses. This gives a lower error at high SNR. Its `t_s` values carry a
  constant offset, which cancels in rates.
- `ksigma` is the detection threshold in noise sigmas after the 400 Hz
  high-pass. Lower it for weak pulses and raise it if noise produces false
  pulses.

### 2.6 `quartz restart`

```
> quartz restart
CLI: quartz restarted mode=motor
> quartz restart          (in mechanical mode)
CLI: ERR nothing to restart in mechanical mode (quartz mode motor|lcd|fork)
```

### 2.7 `quartz pulses`

This lists the most recent motor pulses, oldest first, so the newest appears
next to the prompt. It works in motor mode only.

```
> quartz pulses 4
CLI: quartz pulses n=4 (oldest first)
  seq=27 t_s=27.000412 pol=- peak=511230 snr=51.6 extras=0 clipped=0 closed=1
  seq=28 t_s=28.000409 pol=+ peak=509870 snr=51.5 extras=0 clipped=0 closed=1
  seq=29 t_s=29.000780 pol=- peak=512004 snr=51.7 extras=0 clipped=0 closed=1
  seq=30 t_s=30.000777 pol=+ peak=510118 snr=51.5 extras=0 clipped=0 closed=0
> quartz pulses 40
CLI: ERR usage: quartz pulses [1..16]
```

| Field | Meaning |
|---|---|
| `seq` | Pulse number since the measurement started |
| `t_s` | Leading-edge time since the start, in seconds (sub-sample) |
| `pol` | Polarity of the leading spike; it alternates between pulses |
| `peak` | Leading-spike peak after the high-pass, in codes |
| `snr` | Peak divided by the noise sigma |
| `extras` | Detection or correction bursts in the pulse's group (a fault indication) |
| `clipped` | A raw code reached the rail |
| `closed` | The pulse's 50 ms quiet gap has passed, so `extras` is final |

### 2.8 Help

```
> help quartz
CLI: help quartz (qz)
  Quartz watch timing: motor / LCD / fork
  Usage: quartz [status|mode|gate|fork|ts|ksigma|restart|pulses|help] [arg]
  Quartz-watch timing on the PCM1808 build. Needs a verified
  48 kHz capture. Rates in s/d, + = watch gains, corrected by
  the timebase ppm (UNCAL until one is set: see 'help timebase').
  ...
  Try: quartz help gate | quartz help inhibition | quartz help jumpers
```

`quartz help <sub>` works for `status`, `mode`, `gate`, `fork`, `ts`, `ksigma`,
`restart` and `pulses`, and for the topics `inhibition` (the §5 table) and
`jumpers` (JP1/JP2). The figures that help prints come from the engine
constants: the gate range, the kSigma range, the fork list, the weak-signal SNR
and the no-inhibition span.

Implementation: `src/cli/cli_commands.cpp` (`cmdQuartz` and its subcommand
table). The pure parsers and formatters are in `include/cli/quartz_format.h`
and are host-tested by `tests/test_quartz_cli_format.cpp` (run by
`tests/build_quartz_tests.sh`).

## 3. HID

HID / WebHID: the vendor-HID JSON control plane (`src/hid_control.cpp`,
encoders in `src/hid_json.cpp`) and the browser page `docs/hid/example.html`.
The normative protocol is `docs/HID_CONTROL_PLANE.md` (§5.3 quartz, §8.5
`DspQuartz`, §8.6 `DspQuartzPulse`); TypeScript types are in
`docs/hid/timegrapher_hid.d.ts`. The additions are backwards compatible, so the
protocol version stays 1.

### 3.1 Protocol summary

| Message | What it does | CLI equivalent |
|---|---|---|
| `hello` → `caps.quartz`, `quartzHzMax`, `quartzPulsesMax` | `quartz` is `false` on builds without PCM1808 | — |
| `getState` → `quartz {mode, gateS, forkHz, tsMethod, kSigma}` | Current settings; present only when `caps.quartz` | `quartz mode/gate/fork/ts/ksigma` (show) |
| `patchState` `{"quartz":{...}}` | Merge-patch the settings. `gateS: 0` = auto (until 2 events). `tsMethod` is `"threshold"` / `"xcorr"` (or 0 / 1). A bad value replies `badParam` naming the field | `quartz mode/gate/fork/ts/ksigma <v>` |
| `getQuartz` | Full snapshot: state, reason, headline `rateSDay`, calibration, and the `motor` / `lcd` / `fork` object of the active mode (the other two are `null`) | `quartz status` |
| `getQuartzPulses` `{max}` | Up to 8 recent motor pulses, newest first | `quartz pulses` (up to 16, oldest first) |
| `quartzRestart` | New measurement in the active mode; replies with the snapshot. `badRequest` in mechanical mode | `quartz restart` |
| `subscribe` `{quartzHz, quartzPulses}` | `quartz` notify at up to 5 Hz; one `quartzPulse` notify per motor pulse once its group closed. No history replay | — |

Field names follow the C struct in camelCase (`rate_with_s_day` →
`rateWithSDay`); enums are their stable names (`state: "waitTimebase"`,
`motor.inhibition: "may_include_inhibition"`, `lcd.status: "tracking"`).
Sample indices are decimal strings.

Example: select motor timing with a 60 s gate, then read the result.

```json
{ "v": 1, "id": 4, "op": "patchState", "patch": { "quartz": { "mode": "motor", "gateS": 60 } } }
{ "v": 1, "id": 5, "op": "getQuartz" }
```

```json
{ "v": 1, "id": 5, "ok": true, "result": {
  "mode": "motor", "state": "measuring", "measurementId": 3, "reason": "", "sourceHz": 48000, "timebaseOk": true,
  "elapsedS": 23.4, "gateS": 60, "forkHz": 360, "tsMethod": "threshold", "kSigma": 8,
  "sampleLossRestarts": 0, "rateSDay": 0.9022, "rateValid": true,
  "calPpm": 0, "calSource": "none", "calibrated": false,
  "motor": { "valid": true, "pulses": 23, "rateWithSDay": 0.9022, "rateWithoutSDay": 0.9022,
             "inhibition": "may_include_inhibition", "...": "..." },
  "lcd": null, "fork": null } }
```

Sizes: the `getQuartz` reply is about 0.9 KB, at most 1346 bytes, and
`getQuartzPulses` is at most 1785 bytes. Both fit the 2048-byte HID message
limit; `tests/test_hid_json.cpp` checks this.

### 3.2 Browser page

`docs/hid/example.html` has a **Quartz timing** panel between the device/beat
row and the DSP panels:

- Controls: mode; for motor, the gate (10 s, 60 s, until 2 events, or a custom
  3–960 s), the timestamp method and kSigma; for fork, the frequency. Every
  change is one `patchState`. **Restart measurement** sends `quartzRestart`.
- Headline: the signed rate in s/d (+ = gaining) and a state badge. When
  `calibrated` is false the rate turns amber and an **UNCALIBRATED** warning
  links to the Timebase calibration panel.
- Motor: the §5 status in the same words as the CLI (with 60 s / until 2 events
  buttons when the status is "may include inhibition"); rates with and without
  inhibition (and uncal); the inhibition period; the last and mean jump in µs
  with *n* at 8192 Hz; pulses, step, span, residual, polarity offset, peak SNR,
  noise and threshold, extras and clipping; the weak-signal and JP1 hints.
- LCD and fork: status, f0, harmonics, blocks and retries, or the nominal
  frequency, with both rates and the clipping hint.
- A recent-pulses table (sequence, t, Δt, polarity, SNR, flags), fed by
  `quartzPulse` pushes or by the **poll 1 s** option, keyed by
  (`measurementId`, `sequence`) and cleared when `measurementId` changes.
- A "refitting…" note while `motor.rebuilding` is true.
- The Subscribe panel has `quartzHz` and `quartzPulses`.

Outside mechanical mode the Beat / timing panel is dimmed, and in mechanical
mode the quartz readout is dimmed. Both stay usable. On builds without quartz,
the controls are disabled and a note explains why.

## 4. Display

In any quartz mode the 128×128 OLED swaps the mechanical readout for a quartz
screen; `quartz mode mechanical` brings the mechanical readout back unchanged.
The layouts (ASCII mock-ups) and the order in which fields win space are in
[DISPLAY.md, Quartz screens](DISPLAY.md#quartz-screens).

**Header.** The mode tag (`MOTOR`, `LCD` or `FORK`), an inverse **`UNCAL`**
badge when no timebase calibration is applied, and the GPS badge in PPS builds.
`UNCAL` matters at quartz accuracy: the uncorrected board clock can be off by
about ±1.7 s/d (see [TIMEBASE_CALIBRATION.md](TIMEBASE_CALIBRATION.md)).

**Status line** (above the headline), in order of precedence: `no host
stream` (USB microphone mode) or `stopped` (standalone) while nothing is
running, `clock check...`, `unsupported rate` (capture is not 48 kHz),
`acquiring...`, then for the motor the handoff §5 status: `no inhibition`,
`may incl. inhib.` (short gate, use a longer one), `need 1 more cycle`,
`correcting...` (brief) or `corrected`. LCD and fork show `measuring`. A
finished (`done`) motor result stays on screen after the host stops recording.

**Headline.** The rate in s/d, positive = gaining, with 3 decimals below
10 s/d. In motor mode it is the rate *with* inhibition once the status is
`corrected`, and the raw rate before that.

**Motor rows.**

| Row | Meaning |
|---|---|
| `raw` | Rate without inhibition (the oscillator), s/d |
| `inhib period` | Median spacing of the inhibition events, s |
| `jump` | Last inhibition jump in µs, with n = pulses deleted at 8192 Hz (`244.1µs n=2@8192`) |
| `gate` | Elapsed / gate (`23/60s`), elapsed only (`23s`) when the gate is "until two events", `done` when frozen |

**LCD and fork rows.** `pattern f0` (LCD pattern rate, `--` until the 2 s
pattern search succeeds) or `f nom` (selected fork frequency), `blocks` (phase
blocks in the fit; a rate needs two) and `time` (seconds since the start;
these modes have no gate). The LCD screen also notes that the LCD shows the
oscillator rate before inhibition, so it can legitimately differ from the
motor rate of the same watch.

**Hints** (handoff §8), as inverse bars at the bottom:

- `CLIP: open JP1`: the LEFT pickup hit the ADC rail in motor or fork mode.
- `CLIP: open JP2`: the RIGHT pickup hit the rail in LCD mode.
- `WEAK: reposition`: motor pulses below about 30 µV; move the watch on the
  coil.

Timing stays usable while clipped, but the pulse-shape estimates do not. A
clip hint takes the place of the LCD note.
