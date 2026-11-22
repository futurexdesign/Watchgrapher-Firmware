# Timebase calibration

The board times watches against its own 12 MHz crystal. A crystal error of 1 ppm becomes a rate error of **0.0864 s/d**. A typical uncalibrated crystal is off by 10–30 ppm, which is 1–2.6 s/d: more than the resolution of the rate fit. This page covers how to measure that error and remove it.

## One crystal drives every clock

| Clock | Derived from |
|---|---|
| `time_us_64()` µs timer | XOSC → clk_ref → TICKS |
| PCM1808 SCKI (MCLK, 12.288 MHz) | XOSC → PLL_SYS 150 MHz → GPOUT0 fractional divider |
| ADC sample clock | XOSC → PLL_USB 48 MHz → clk_adc |

So **one ppm figure corrects every capture path**, and measuring the µs timer is the same as measuring the audio sample clock. The firmware holds that figure in the `timebase` module. `beat_rate` divides it out of every rate, beat-error, amplitude and jitter result. The sign convention is **+ppm = board clock fast**.

## Options

| Method | Accuracy | Needs | Status |
|---|---|---|---|
| Host NTP-disciplined clock (`scripts/calibrate_timebase.py`) | 0.05–0.3 ppm in 30 min, below 0.1 ppm in 60 min, limited by how well the host's NTP daemon holds frequency | Nothing; it uses the USB CDC link | **Implemented. Recommended.** |
| Host clock via the WebHID page (`docs/hid/example.html`, "Timebase calibration") | Same method, but browser clocks are coarse (`Date.now()` is rounded to 1 ms) and background tabs are throttled. Expect about 0.3–1 ppm in 30 min in practice | A Chromium browser | **Implemented.** The convenient option |
| Direct NTP queries (`--ntp-server`) | Each query is ±1–10 ms, so about 3 ppm/h and about 0.1 ppm/day | Network, `ntplib` | Implemented as a **cross-check** |
| GPS 1 PPS (`-DPPS_GPIO=N`) | Better than 0.01 ppm after 64 s, and it tracks temperature continuously | 1 GPIO (0–31), a GPS module with PPS output; optionally a second pin for its NMEA output (`-DGPS_UART_RX=N`) to gate PPS on a valid fix | **Implemented, optional.** On in `pico2_pcm1808` (`-DPPS_GPIO=3`); compiled out of every other env. The NMEA input is compiled out of all of them |
| TCXO instead of the crystal | ±0.5–2.5 ppm over temperature with no calibration; about 0.1 ppm after one calibration | A board change | Recommended for the next board revision |
| USB SOF / the USB clock | **None.** SOF timing comes from the host controller's own crystal (USB allows ±2500 ppm; real controllers are about ±20–50 ppm). NTP disciplines the OS clock in software, never that crystal | — | Rejected |
| Temperature-sensor compensation | About ±1–5 ppm across the crystal's curve, but each board needs characterising | — | Not worth it. Calibrate warm instead |
| A quartz watch as the reference | ±5–15 ppm, no better than the board | — | Rejected |

## Calibrating against NTP

1. **Check that the host clock is synced to NTP.** The script reports what it can detect (`timedatectl` on Linux, `systemsetup` on macOS, `w32tm` on Windows).
2. **Warm the board up.** Power it for about 10 minutes, ideally streaming audio, so the crystal is at operating temperature.
3. **Run the measurement:**

   ```sh
   pip install pyserial            # plus ntplib for --ntp-server
   python3 scripts/calibrate_timebase.py --port /dev/cu.usbmodem101 --duration 1800
   ```

   While it runs:
   - it polls `time` every 2 s and keeps the lowest round-trip sample in each 60 s bucket;
   - it prints a running estimate every minute;
   - if the host clock steps, it restarts the measurement.

   Run it twice. The two results should agree to within about 0.5 ppm.
4. **Apply and persist the result.** Close any audio stream first, because a flash write stalls the CPU for tens of ms (`timebase save` is refused while the stream is open; on a standalone device with no stream it pauses capture for the write and restarts the measurement):

   ```sh
   python3 scripts/calibrate_timebase.py --port ... --duration 1800 --apply --save
   ```

   or set it by hand:

   ```
   timebase set -23.412 ntp 1763000000 1800
   timebase save
   ```

5. **Recalibrate occasionally.** Crystals age by a few ppm per year, and the calibration only holds near the temperature it was measured at (an AT-cut crystal stays within about ±1 ppm across 20–30 °C).

The record stored in flash is versioned and CRC-checked, and is loaded at boot. It lives in the Arduino-Pico EEPROM-emulation sector, at the end of flash, in the first 256 bytes of a 1 KB image shared with the device settings and the calibration log (see "Flash layout" below).

### From the WebHID page

`docs/hid/example.html` has a **Timebase calibration** panel. It shows the correction in effect and what is stored (including GPS PPS status), and has buttons to set a value by hand, save it to flash, or clear it.

**Measure** runs the same fit as the Python script inside the browser. It polls `getTime`, uses `Date.now()` as the reference, keeps the lowest-RTT sample per bucket, and restarts if the clock steps. When it finishes, **Apply result** sends the value to the device (source `ntp`, RAM only) and **Save to flash** persists it.

Keep the tab visible for the whole run: browsers throttle timers in hidden tabs, and the page warns if that happened. On synthetic data with 1 ms clock rounding the page's fit recovers 23.4 ppm to within 0.02 ppm in 30 min. Real browsers add their own scheduling jitter, so use the Python script when you need the best result.

### Other ways to set it

- **CLI.**
  - `timebase` shows the current state.
  - `timebase set <ppm> [manual|ntp] [unix_date] [duration_s]` applies a value; `save` persists it and `clear` resets it.
  - `dsp set beat cal_ppm <ppm>` is the same store with the source `manual`.
- **HID.** Use `getTimebase`, `setTimebase`, `saveTimebase` and `clearTimebase`, plus `getTime` for host tools (see [HID_CONTROL_PLANE.md](HID_CONTROL_PLANE.md)). Every successful save is also recorded in the calibration history log (below).
- **From a reference.** If a reference known to be exact reads *R* s/d, the correction is `ppm = −R / 0.0864`.

## GPS PPS (optional)

Build with `-DPPS_GPIO=N`, for example by adding it to the env's `build_flags` (`pico2_pcm1808` already sets `-DPPS_GPIO=3`; `kiwi_pcm1808` does not, because GP3 is its LRCK). Wire the GPS module's 1 PPS output (3.3 V logic) to GPIO N, with a common ground. The pin must be GPIO 0–31.

**Measurement.**
- A PIO state machine (`scripts/pps_counter.pio.in` → `include/pps_counter.pio.h`, regenerated with `python3 scripts/gen_pps_pio.py`) counts clk_sys cycles between PPS rising edges in a two-cycle loop.
- There is **no interrupt and no DMA**: `timebase_poll()` in `loop()` drains the FIFO.
- Edge-sampling error is ±2 cycles (13 ns) per edge and does not accumulate.

**Filtering and lock.**
- A period more than 200 ppm from nominal is rejected; that covers a missed pulse or a glitch. It does **not** catch a module that keeps pulsing from its own oscillator without a fix: those pulses are regular. Add the NMEA input (below) to catch that.
- Accepted periods are averaged over a sliding 64 s window (`include/pps_filter.h`).
- After 16 good pulses PPS is **locked**. It then overrides the stored calibration live, with source `pps`.
- After 10 s with no good pulse the window clears and the stored value applies again.

`timebase` (and HID `getTimebase.pps`) shows the lock state, the PPS ppm, the window size, and the good/reject counts. To store the PPS-derived value for use without the GPS, set it with `timebase set <ppm> manual` and then `save`.

### GPS NMEA input (optional, recommended with PPS)

Build with `-DGPS_UART_RX=N` as well and wire the module's TX (3.3 V TTL serial) to GPIO N. Only the receive line is used; nothing is sent to the module.

| Flag | Default | Meaning |
|---|---|---|
| `GPS_UART_RX` | (off) | RX pin. Must be a UART RX pin: GP1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29 (and 31–47 on RP2350B). **GP9** (UART1) is free on both boards |
| `GPS_UART_BAUD` | 9600 | Module baud rate (most modules default to 9600) |
| `GPS_MIN_SATS` | 4 | Satellites in use required for a valid fix |

- **Fix gating.** PPS pulses only count while the receiver reports a fix: the latest RMC is less than 3 s old with status `A` (and mode not `N`), and, if the module sends GGA, the fix quality is above 0 with at least `GPS_MIN_SATS` satellites. Without a fix, pulses are discarded (`ignored_no_fix` / HID `pps.noFix`) and the averaging window is cleared, so lock needs 16 fresh pulses after the fix returns. The OLED GPS badge stays an outline until then.
- **Date stamping.** A calibration set without a date (`timebase set -23.4 manual`, `dsp set beat cal_ppm`) gets the GPS UTC date.
- **What it doesn't do.** NMEA sentences arrive tens to hundreds of ms after the second they describe, so they play no part in the frequency measurement; that comes only from the PPS edges.
- **UART choice.** UART0 pins (GP1, 3, 13, 15, 17, 19, 29) clash with `DIAG_UART`, which uses Serial1 on UART0; the build rejects that combination. The receiver is polled from the 32-byte hardware FIFO (about 33 ms of data at 9600 baud); `overruns` counts characters lost if `loop()` ever stalls longer than that.
- Parsing is in `include/nmea.h` (any talker: GP, GN, GL, …; checksums required), host-tested by `tests/test_nmea.cpp`.

`timebase` shows a one-line GPS summary, then the log state:

```
CLI: timebase gps fix=yes quality=1 sats=9 hdop=0.92 utc=1794746119 last_rmc=412ms sentences=5310 errors=0 overruns=0
CLI: timebase log logdelta_ppm=0.200 pending=no (see: timebase log)
```

`timebase gps` prints the full detail (the same data as HID `getGps`): fix, DOPs, position and the satellite table. Without `-DGPS_UART_RX` it prints `CLI: ERR GPS not built in (build with -DGPS_UART_RX=<gpio>)`.

```
CLI: timebase gps fix=yes quality=1 sats_used=9 utc=2026-11-15 12:35:19Z (unix 1794746119) last_rmc=412ms
CLI: timebase gps hdop=0.92 pdop=1.40 vdop=1.05
CLI: timebase gps pos lat=47.6062000 lon=-122.3321000 alt_m=56.0
CLI: timebase gps sats n=12
  sys  prn   el    az  snr used
  GP     5   62   120   44 yes
  ...
```

`timebase log` prints the history, oldest first:

```
CLI: timebase log entries=3/24 pending=no logdelta_ppm=0.200 last_write=2026-11-15 12:35:19Z
   # date (UTC)                 ppm source  duration_s reason
   0 2026-09-21 14:13:20Z    -23.412 ntp          1800 save
   ...
```

## Calibration history log

The device keeps a history of its clock calibrations, so a host can see how the crystal drifts over weeks and months without anyone keeping notes. The log is a **ring of 24 entries** in flash; when it is full the oldest entry is dropped. Each entry holds the date (UTC seconds), the ppm, the source (`manual`, `ntp` or `pps`), the measurement duration and the reason it was logged. It is shown by `timebase log` (CLI) and `getTimebaseLog` (HID, paged; see [HID_CONTROL_PLANE.md §5.8](HID_CONTROL_PLANE.md)). It is advisory only: the correction applied to timing always comes from the stored calibration or the live PPS value, never from the log.

### What gets logged

| Reason | Trigger |
|---|---|
| `save` | Every successful `timebase save` / `saveTimebase` (manual or NTP calibration). The entry is written in the **same flash commit** as the calibration. |
| `delta` | A GPS PPS measurement that differs from the **last entry (of any source)** by at least `logDeltaPpm`, **and** the last entry is at least one day old. The very first PPS entry (empty log) is also `delta`. |
| `monthly` | A GPS PPS measurement when the last entry (of any source) is at least 30 days old, whatever the ppm. Wins over `delta`. |

A PPS measurement is only a **candidate** when all of these hold (constants from `include/timebase_log.h`, decision in `timebase_log_pps_decision`):

| Constant | Value | Meaning |
|---|---|---|
| `TIMEBASE_LOG_PPS_WINDOW_FULL` | 64 pulses | The PPS average window is full (a lock needs only 16; the log wants the full 64 s average) |
| `TIMEBASE_LOG_PPS_MIN_LOCK_S` | 600 s | PPS has been **continuously** locked for at least 10 minutes (the streak restarts whenever lock is lost) |
| (GPS UTC) | known | The entry is dated by GPS UTC; with no UTC there is **no PPS entry** |
| `TIMEBASE_LOG_MIN_DELTA_AGE_S` | 86 400 s (1 day) | Minimum spacing between `delta` entries |
| `TIMEBASE_LOG_MONTHLY_S` | 2 592 000 s (30 days) | Spacing that forces a `monthly` entry |
| `TIMEBASE_LOG_DELTA_PPM_MIN` / `_MAX` | 0.05 / 10 ppm | Accepted range of the user-set threshold `logDeltaPpm` |
| `TIMEBASE_LOG_DELTA_PPM_DEFAULT` | 0.2 ppm | Default threshold |
| `TIMEBASE_LOG_CAP` | 24 entries | Ring size |

The policy is evaluated once per second while PPS is locked, and not while an entry is still pending, so one trigger yields one entry. If the clock goes backwards (UTC earlier than the last entry) no entry is made, unless the stored date is more than a day in the future (it was dated by a bad clock): then the age test is waived and only the ppm test applies.

### Why so conservative: flash wear

A calibration entry costs a **flash sector erase**, and the flash is rated for a limited number of erase cycles per sector (typically about 100 000 for the QSPI NOR parts used with the RP2350; check your part's datasheet). A PPS lock produces a fresh, slightly different ppm every second, and the crystal wanders with temperature, so logging every change would burn the sector within days. The thresholds bound the write rate:
- a full 64-pulse average and 10 minutes of continuous lock reject warm-up transients and short-term noise;
- `delta` needs a real change (`logDeltaPpm`, default 0.2 ppm = 0.017 s/d) **and** at least one day since the last entry, so at most about one `delta` write per day;
- `monthly` gives a heartbeat even for a perfectly stable crystal (a flat history is also information);
- the log has no other writer: at most one write per day (plus the manual `save`s) is more than 270 years of margin at 100 000 cycles.

### When the write happens (deferred write)

An appended entry is first held **in RAM** and marked `pending` (`timebase log` shows `pending=yes`, HID `pending` / `Timebase.logPending`). Writing flash stalls execute-in-place for tens of milliseconds, which would corrupt a live USB isochronous stream or a capture, so the pending entry is written only when it is safe:
- **never while the USB audio stream is open**;
- **not while a quartz measurement is acquiring or measuring** (the pause would restart it); a finished (`done`) or idle measurement does not block it;
- otherwise at most one attempt per **10 s** (**60 s** after a failed or refused attempt). The write pauses capture, erases and reprograms the sector, verifies it by reading it back, and restarts capture (a fresh stream: DSP and quartz state reset).

`lastWriteUnix` is the GPS UTC of the last successful write (0 = none or no UTC). A reboot before the write loses the pending entry (it was only in RAM). `timebase log clear` and `saveTimebase` are **not** deferred: they write at once and are refused (`busy`) while a USB stream is open.

### Commands

| | CLI | HID |
|---|---|---|
| Show the log | `timebase log` | `getTimebaseLog` (`offset` pages: 24 entries need at most 2) |
| Erase the log | `timebase log clear` | `clearTimebaseLog` |
| Show / set the threshold | `timebase logdelta [ppm]` (0.05..10, saved to flash) | `getTimebase.logDeltaPpm`; `setTimebase { "logDeltaPpm": x }` (`ppm` may be omitted) |
| Pending write | `pending=` in `timebase log` | `TimebaseLog.pending`, `Timebase.logPending` |
| GPS detail (to judge a PPS entry) | `timebase gps` (fix, UTC, DOP, position, satellite table) | `getGps` |

Setting `logdelta` writes flash (the device-settings record), so it is refused while a USB stream is open.

### Flash layout

Arduino-Pico emulates EEPROM with one 4 KiB flash sector; the logical image is **1024 bytes**, held in RAM and written back as a whole. A single module, `flash_store` (`include/flash_store.h`), owns `EEPROM.begin` / `commit`, so two features can never commit over each other with stale data:

| Region | Offset | Size | Owner and content |
|---|---:|---:|---|
| TIMEBASE | 0 | 256 B | Clock calibration record (`timebase.cpp`): magic, version, ppm, source, date, duration, CRC. **Unchanged** from earlier firmware |
| DEVICE | 256 | 256 B | Device settings (`device_mode.cpp`): magic `DEV1`, version, size, device mode, `logDeltaPpm`, CRC |
| TBLOG | 512 | 512 B | Calibration log (`timebase_log.h`): a 404-byte record, magic `TBL1`, version, size, `count`, `head`, 24 × 16-byte entries (u32 date, float ppm, u8 source, u8 reason, u16 reserved, u32 duration), CRC-32 over the preceding 400 bytes |

Every record is CRC-checked; erased flash (`0xFF`) fails the magic check and reads as "nothing stored" (defaults: `usbMic`, 0.2 ppm, empty log). Growing the image from 256 to 1024 bytes leaves the old calibration record where it was, so upgrading keeps the stored ppm. A commit that changes no byte does nothing, and commits are refused while the USB audio stream is open.

## Hardware: TCXO

For a new board revision, replace the 12 MHz crystal with a 12 MHz clipped-sine or CMOS **TCXO** (±0.5–2.5 ppm, 3.3 V, 2016/2520 package, about $1–3). Feed it into XIN with the XOSC in external-clock mode, or follow the RP2350 hardware design guide's guidance on external clocks. That removes most of the temperature drift. One NTP calibration then holds to about 0.1–0.5 ppm.
