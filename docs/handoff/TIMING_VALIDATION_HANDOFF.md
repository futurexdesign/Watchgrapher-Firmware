# Handoff: timing validation with the ChronoPulse recordings

**For:** an agent (or person) validating the mechanical timing engine on a real board.
**State:** written 2026-11-24. The procedure below was run by hand on 2026-11-23 (firmware of that day); the scripted version `scripts/bench/validate_clips_hw.py` packages the same steps but has **not yet been run end to end on hardware**: treat its first run as part of the job and fix it if needed.

## 1. What is being validated, and against what

The stand times a mechanical watch from its tick sound: rate (s/d), beat error (ms) and amplitude (°). To check that on hardware without a pickup or a watch, the firmware's **synthetic source** plays a recorded watch through the real DSP chain in place of the PCM1808 input. The recordings are ten takes from the ChronoPulse app's test set, trimmed into loops.

There are three references, and they mean different things:

| Reference | Where | What it is good for |
|---|---|---|
| **Designed loop rate** | `assets/synth_clips/clips.json`, `expected.rate_s_day` | **The rate truth for the loop.** Each loop holds an even number of beats and is exactly `round(N × fitted half-period × 48 kHz)` samples long, so a correct timegrapher must read this rate. Use it for pass/fail on rate |
| **ChronoPulse** | `clips.json`, `chronopulse` (from `ChronoPulse_Results.txt`: lift angle 52°, 20 s integration, measured on the **whole** recordings) | Rate, beat error, amplitude (onset and peak) of an established instrument. Its rate can differ from the loop's by 1–3 s/d where the watch drifted during the recording (NH35, Starbucks, hulk, Waltham): that is the recording, not an error. Use it for amplitude and beat error, which the loops preserve |
| **Generated scenarios with exact truth** | `assets/synth_clips/generated/*.wav` + `*.truth.json` | Exact rate, signed beat error and per-side amplitude, plus impairments (noise, hum, clipping, knocks, dropouts, a watch change). Host only: they are 20 s files, not seamless loops (see §6) |

## 2. Materials

| Item | Path | Notes |
|---|---|---|
| Trimmed loops (mono 16-bit, 48 kHz) and their numbers | `assets/synth_clips/*.wav`, `clips.json`, `README.md` | Nine whole-beat loops (3.7–4.4 s) and `switch_21600_28800` (29 s, a watch change from 21,600 to 28,800 bph, not a whole-beat loop) |
| How the loops were cut | `scripts/synth_clips/make_clips.py`, `analyze.py` | Re-run from the original recordings to regenerate (needs numpy, scipy, soundfile) |
| Original recordings | not in the repo (`ChronoPulse-Test-Audio`, user's copy) | Only needed to regenerate the loops |
| Build hook that turns a clip into the synthetic table | `scripts/synth_clip_header.py` (PlatformIO pre-script) | `-DTG_SYNTH_CLIP=<id>`; unknown id stops the build and lists the known ones |
| Host harness (the real firmware DSP chain on the Mac) | `tests/dsp_host/` (`build.sh`, `dsp_wav`) | Same code as the board, shimmed; fast; the first thing to run |
| Comparison table over all clips | `scripts/synth_clips/compare.py` | Runs the harness (and optionally the reference) and prints rate / BE / amplitude / jitter / lock tables |
| Offline reference of opentimegrapher's method | `scripts/synth_clips/reference_otg.py` | A second opinion with switchable steps |
| Hardware bench client and validator | `scripts/bench/tgclient.py`, `validate_clips_hw.py`, `beatspacked.py` | USB HID (and BLE) client; per-clip flash + measure |
| Recorded baseline (before the timing-engine work) | `tests/dsp_host/baseline_firmware.json` | For before/after |

## 3. Setup

- **Board:** Raspberry Pi Pico 2 W, connected to the Mac by USB. It must appear as `/dev/cu.usbmodem*` (check with `ls /dev/cu.usbmodem*`; if it is absent, PlatformIO may "upload" to another serial device, such as a Bluetooth port, and fail with "No accessible RP2040/RP2350 devices in BOOTSEL mode").
- **Tools:** PlatformIO (`pio`), a C++17 compiler, Python 3 with a venv:
  ```bash
  python3 -m venv .venv && .venv/bin/pip install hidapi bleak numpy scipy soundfile
  ```
- **Env:** `pico2w_synthetic` (synthetic source, Bluetooth on core 1). Any env with `AUDIO_SOURCE=SYNTHETIC` works; `pico2_synthetic` for a plain Pico 2.
- **Nothing else may hold control** of the device: close the web app or `docs/hid/example.html`, or the HID requests get `linkBusy` (the scripts take control with `claim {takeover: true}`, which kicks the other app off).

## 4. Procedure

### 4.1 Host first (minutes, no board)

```bash
tests/run_all.sh /tmp/tg_host                 # the dsp-host-* rows run the real chain over clips
.venv/bin/python scripts/synth_clips/compare.py --json /tmp/now.json \
    --baseline tests/dsp_host/baseline_firmware.json
.venv/bin/python scripts/synth_clips/compare.py --harness-args "--timing drop" --no-otg
```

Every `dsp-host-*` row must pass. The tables give the per-clip numbers the board should reproduce: the board runs the same code on the same samples, so **host and board results should agree to within the averaging noise** (a few hundredths of a s/d on rate, tenths of a degree on amplitude). A disagreement means a hardware-only effect (CPU overrun, lost blocks, clock), which is exactly what the hardware run is for.

### 4.2 Hardware, scripted

```bash
.venv/bin/python scripts/bench/validate_clips_hw.py --timing both --out /tmp/hw.jsonl --restore-default
```

Per clip (about 2.5 min each with both timing modes; ~25 min for all ten) it:

1. builds and uploads `PLATFORMIO_BUILD_FLAGS=-DTG_SYNTH_CLIP=<id> pio run -e pico2w_synthetic -t upload` (upload uses the USB CDC 1200-baud reset into BOOTSEL; no button press);
2. reconnects over HID, checks `hello.source == "SYNTHETIC:<id>"`;
3. saves the beatRate parameters, then sets `liftAngle 52`, `avgS 20`, `timingLandmark` 0 (unlock) or 1 (drop), and **reads them back**;
4. `setDevice {mode: "standalone"}` and `start`. **This matters:** in USB microphone mode the timing engine is off by design (`getBeat.analysis == "off"`, only the beat-rate lock runs);
5. `resetTiming`, waits 48 s (lock, provisional readings from ~6 s, `settled` at 10 s of stretch, then a full 20 s average), samples `getBeat` five times 2 s apart and takes medians, then reads `getTiming`;
6. restores the saved parameters; for `switch_21600_28800` it instead logs lock and average every second for 75 s.

It prints one JSON line per clip and mode (also appended to `--out`) and a markdown table. `--clips <id> ...` limits the run; `--no-flash` measures the firmware already on the board (one clip).

### 4.3 Hardware, by hand (what the script automates)

Useful when the script needs debugging. With `scripts/bench` on `sys.path`:

```python
from tgclient import HidClient
c = HidClient(); c.request("claim", takeover=True)
c.request("hello")["result"]["source"]                      # "SYNTHETIC:nh35_21600"
r = c.request("patchState", patch={"blocks": {"beatRate": {"params":
        {"liftAngle": 52, "avgS": 20, "timingLandmark": 1}}}})
assert r["ok"], r                                           # the "patch" wrapper is required
c.request("setDevice", mode="standalone"); c.request("start")
c.request("resetTiming")
# ... wait ~48 s ...
b = c.request("getBeat")["result"]; b["avgLong"], b["settled"], b["timingSource"]
c.request("getTiming")["result"]["dspBlockUsMax"]
```

## 5. Pass criteria

| Check | Pass |
|---|---|
| Lock | `locked`, `vphNearest` = the clip's bph, `state "locked"`, `settled true`, `analysis "on"` |
| Rate | within **±0.3 s/d of the designed loop rate** (`expected.rate_s_day`), both timing modes. Drop timing should be closer (it was within ±0.07 on 9 of 10 clips on the host) |
| Amplitude | within **±15° of ChronoPulse's onset figure** (the firmware's lift landmarks track ChronoPulse's onset method; it read −4…+16° from it), and within ~1° of the host result for the same mode |
| Beat error | informational against ChronoPulse (instruments disagree by 0.1–0.4 ms on these clips); must match the host result for the same mode within ~0.05 ms |
| Host = board | every figure within the tolerances above of the host table from §4.1 for the same mode and settings |
| CPU | `getTiming.dspBlockUsMax` < 5,000 µs (the per-block budget) and `loopUsMax` < ~8,000 µs (the USB audio slack). **Known failure, see §7** |
| Watch change (`switch_21600_28800`) | lock moves 21,600 → 28,800 (and back at the loop wrap) without readings that mix the two watches: no settled average differs from ≈ +3.3 s/d by more than 2 s/d while locked at 28,800 |

## 6. Pitfalls learnt the hard way

- **`patchState` needs the `patch` wrapper.** `request("patchState", blocks=…)` answers `badRequest` ("missing patch object") and nothing changes. A first validation run on 2026-11-23 silently ran at the wrong lift angle because the reply wasn't checked. Always assert `ok` and read the parameters back.
- **Parameters reset on every reflash** to the firmware defaults (lift angle 51°, `avgS` 10, `timingLandmark` 0). Flashing a clip build therefore changes the user's settings; the script restores what it found, but the defaults are what it finds after a reflash.
- **USB microphone mode turns the timing engine off** (by design since 2026-11-23). Measure in standalone mode.
- **Compare rate with the loop, not with ChronoPulse.** ChronoPulse measured the whole recordings; several watches drift by 1–3 s/d within them, and the loop reproduces the rate fitted over the whole recording.
- **Drop timing needs a learning period:** 16 beats per side for the templates; readings start on unlock timing and hand over to drop timing at about 6.5 s (`timingSource`, `handovers`).
- **Generated scenarios are host-only.** They are 20 s files, not whole-beat loops; looped on the board, the wrap every 20 s is a timing jump (a cut) and the readings restart each time.
- **Clipped recordings** (`waltham`, `st3600`, `hulk`, `freesprung`, `switch` peak at exactly 0.100): they exercised and fixed an amplitude bug (pulse strength by 0.5 ms mean); keep them in the set.
- **Flash size:** a 4 s clip adds about 345 KB, the switch clip about 2.8 MB; only the selected clip is linked.
- **Host timing figures are noisy** (other processes on the Mac); judge CPU on the board with `getTiming`.

## 7. Last known results (for orientation, not as truth)

Board, 2026-11-23, unlock timing, lift 52°, 20 s average, firmware of that day (before items 2/3 of the timing-engine plan):

| Clip | Loop rate | Board rate | ChronoPulse rate | Board amp | ChronoPulse onset / peak |
|---|---|---|---|---|---|
| waltham_18000 | +20.05 | +20.06 | +20.9 | 307 | 309 / 287 |
| m8215_incase_21600 | +3.93 | +3.99 | +4.1 | 293 | 287 / 278 |
| m8215_jubilee_21600 | −15.78 | −15.79 | −15.5 | 311 | 300 / 279 |
| nh35_21600 | +4.05 | +4.29 | +5.5 | 267 | 261 / 254 |
| nh39a_21600 | +17.19 | +17.19 | +17.3 | 335 | 328 / 318 |
| st3600_21600 | +12.05 | +12.04 | +11.9 | 328 | 327 / 292 |
| r3135_hulk_28800 | +15.30 | +15.32 | +16.1 | 297 | 301 / 276 |
| r3235_freesprung_28800 | +5.28 | +5.39 | +5.2 | 256 | 240 / 232 |
| r3235_starbucks_28800 | +4.50 | +4.57 | +2.3 | 228 | 214 / 206 |

Board, same day, `nh35_21600`, drop timing: rate **+4.052** (loop +4.05), jitter 51 µs (unlock: 388 µs), amplitude 266.7°. **CPU: worst DSP block 5,517 µs in drop mode** (unlock 1,673 µs) against the 5,000 µs budget: a known failure, owned by the timing-engine plan's performance task (`docs/plans/TIMING_ENGINE_PLAN.md`, status in §6/§8). Re-measure after that task.

Host (current tree, drop timing, items 1–4 and 8 done): mean absolute amplitude error against ChronoPulse onset 4.3° (was 6.7°), beat error 0.086 ms (was 0.13 ms); rate within ±0.07 s/d of the loop on 9 of 10 clips (`st3600` +0.20).

## 8. Reporting

Record the run in `docs/acceptance_results.md` (new dated section): firmware state (the tree is uncommitted; note the date and `git status` summary), env, settings, the table from the script, the CPU figures, and any deviation from the host table with its explanation. Keep `/tmp/hw.jsonl` as the raw data. If a check fails, say which, with numbers; do not loosen a tolerance without saying so.
