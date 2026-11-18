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
| GPS 1 PPS (`-DPPS_GPIO=N`) | Better than 0.01 ppm after 64 s, and it tracks temperature continuously | 1 GPIO (0–31), a GPS module with PPS output; optionally a second pin for its NMEA output (`-DGPS_UART_RX=N`) to gate PPS on a valid fix | **Implemented, optional.** Compiled out by default |
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
4. **Apply and persist the result.** Close any audio stream first, because a flash write stalls the CPU for tens of ms:

   ```sh
   python3 scripts/calibrate_timebase.py --port ... --duration 1800 --apply --save
   ```

   or set it by hand:

   ```
   timebase set -23.412 ntp 1763000000 1800
   timebase save
   ```

5. **Recalibrate occasionally.** Crystals age by a few ppm per year, and the calibration only holds near the temperature it was measured at (an AT-cut crystal stays within about ±1 ppm across 20–30 °C).

The record stored in flash is versioned and CRC-checked, and is loaded at boot. It lives in the Arduino-Pico EEPROM-emulation sector, at the end of flash.

### From the WebHID page

`docs/hid/example.html` has a **Timebase calibration** panel. It shows the correction in effect and what is stored (including GPS PPS status), and has buttons to set a value by hand, save it to flash, or clear it.

**Measure** runs the same fit as the Python script inside the browser. It polls `getTime`, uses `Date.now()` as the reference, keeps the lowest-RTT sample per bucket, and restarts if the clock steps. When it finishes, **Apply result** sends the value to the device (source `ntp`, RAM only) and **Save to flash** persists it.

Keep the tab visible for the whole run: browsers throttle timers in hidden tabs, and the page warns if that happened. On synthetic data with 1 ms clock rounding the page's fit recovers 23.4 ppm to within 0.02 ppm in 30 min. Real browsers add their own scheduling jitter, so use the Python script when you need the best result.

### Other ways to set it

- **CLI.**
  - `timebase` shows the current state.
  - `timebase set <ppm> [manual|ntp] [unix_date] [duration_s]` applies a value; `save` persists it and `clear` resets it.
  - `dsp set beat cal_ppm <ppm>` is the same store with the source `manual`.
- **HID.** Use `getTimebase`, `setTimebase`, `saveTimebase` and `clearTimebase`, plus `getTime` for host tools (see [HID_CONTROL_PLANE.md](HID_CONTROL_PLANE.md)).
- **From a reference.** If a reference known to be exact reads *R* s/d, the correction is `ppm = −R / 0.0864`.

## GPS PPS (optional)

Build with `-DPPS_GPIO=N`, for example by adding it to the env's `build_flags`. Wire the GPS module's 1 PPS output (3.3 V logic) to GPIO N, with a common ground. The pin must be GPIO 0–31.

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

Status line:

```
CLI: timebase gps fix=yes quality=1 sats=9 hdop=0.92 utc=1794746119 last_rmc=412ms sentences=5310 errors=0 overruns=0
```

## Hardware: TCXO

For a new board revision, replace the 12 MHz crystal with a 12 MHz clipped-sine or CMOS **TCXO** (±0.5–2.5 ppm, 3.3 V, 2016/2520 package, about $1–3). Feed it into XIN with the XOSC in external-clock mode, or follow the RP2350 hardware design guide's guidance on external clocks. That removes most of the temperature drift. One NTP calibration then holds to about 0.1–0.5 ppm.
