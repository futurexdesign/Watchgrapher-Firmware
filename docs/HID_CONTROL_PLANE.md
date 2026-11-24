# HID control plane (WebHID / JS-first)

**Status:** Implemented (H1–H6 in firmware; H4 example page). The serial CLI is only in `TG_SERIAL_CLI` builds; the CDC port (upload auto-reset) is in every build. H7 (BLE transport and control ownership, §4.6) is in progress: see [`BLE_CONTROL_PLANE.md`](BLE_CONTROL_PLANE.md). This document is normative for hosts and firmware. Where this document and the code differ, the code (`src/hid_control.cpp`, `src/hid_json.cpp`) wins; please report the difference.

**Goal:** Expose the same DSP surface as [`control_api`](../include/control_api.h) to a browser via [WebHID](https://developer.mozilla.org/en-US/docs/Web/API/WebHID_API), without parsing CLI text or C enums. Audio stays on UAC2; HID carries control JSON only.

**Related:** TypeScript types in [`hid/timegrapher_hid.d.ts`](hid/timegrapher_hid.d.ts), browser sketch in [`hid/webhid_example.md`](hid/webhid_example.md), smoke page [`hid/example.html`](hid/example.html), as-built overview in [`AS_BUILT_ARCHITECTURE.md`](AS_BUILT_ARCHITECTURE.md), timebase log policy in [`TIMEBASE_CALIBRATION.md`](TIMEBASE_CALIBRATION.md). The end-user web app is the sibling repository `timegrapher-web` (WebHID, plus a built-in virtual device for development without hardware); it takes its types from this repository's `hid/timegrapher_hid.d.ts`.

---

## 1. Design summary

| Choice | Decision |
|---|---|
| USB composite | Keep UAC2 mic + CDC port; **add** vendor HID (`CFG_TUD_HID=1`). The CLI on the CDC port is only in `TG_SERIAL_CLI` builds (H7) |
| Wire body | **JSON UTF-8**, camelCase keys, string block names |
| Transport | Host OUTPUT report ↔ device INPUT report (no Feature reports for API) |
| Correlation | Request `id` echoed on replies; unsolicited pushes use `"notify": true` |
| Firmware boundary | HID codec → `control_api_*` only (never call `dsp_graph` directly) |
| Audio | Never carried over HID |

```text
Browser (WebHID)
  sendReport(OUTPUT) ──► hid_control (framer + JSON) ──► control_api ──► dsp_graph
  inputreport ◄──────── hid_control (rsp / notify) ◄──── control_api ◄── dsp_graph
```

**Transports.** The framing (§3) and the JSON (§4 onward) do not depend on the link. USB HID is one link. BLE is a second link, specified in [`BLE_CONTROL_PLANE.md`](BLE_CONTROL_PLANE.md) (Pico 2 W only), which uses the same frames and the same ops. Each link has its own reassembly buffer and its own `msgSeq` counter. Only one link owns the control plane at a time (§4.6).

---

## 2. HID identity (WebHID filters)

| Field | Value |
|---|---|
| Usage Page | `0xFFA0` (vendor-defined) |
| Usage | `0x0001` (Timegrapher Control) |
| Report ID | `0x01` (both Input and Output) |
| Report size | 64 bytes including report ID → **63** bytes payload |
| Endpoints | Interrupt IN + Interrupt OUT, full-speed |
| Collection | Application |

### 2.1 Recommended device filter

Prefer usage page/usage so filters survive VID/PID changes:

```js
const devices = await navigator.hid.requestDevice({
  filters: [{ usagePage: 0xffa0, usage: 0x0001 }],
});
```

### 2.2 Current firmware USB IDs (informational)

As of this writing, UAC init sets `TinyUSBDevice.setID(0xCafe, 0x4003)` in `usb_audio.cpp`. Treat VID/PID as secondary filters only; **Usage Page `0xFFA0` / Usage `0x0001` are authoritative** for this control plane.

---

## 3. Framing (binary envelope; JSON payload)

JSON messages (`getState`, event batches) often exceed 63 bytes. Each HID report carries a small header plus a UTF-8 chunk. The reassembled payload is always a single JSON object.

### 3.1 Report layout

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | `reportId` = `0x01` |
| 1 | 1 | `flags` — bit0 = `moreFragments`, bit1 = `notifyHint` (optional advisory) |
| 2 | 1 | `msgSeq` — message counter (wraps); same for all fragments of one message |
| 3 | 1 | `fragIndex` — 0-based |
| 4 | 1 | `fragCount` — total fragments (≥ 1) |
| 5 | 1 | `payloadLen` — bytes of UTF-8 in this report (0..58) |
| 6 | N | UTF-8 chunk (`payloadLen` bytes, N ≤ 58); remainder of the 64-byte report is unused/zero |

Maximum payload bytes per report: **58**. The report is 64 bytes: the report ID (1) leaves a 63-byte buffer, and the 5-byte header (`flags`, `msgSeq`, `fragIndex`, `fragCount`, `payloadLen`) leaves 58 (`64 - 1 - 5`). In the table above the offsets count the report ID as byte 0; the header proper is bytes 1..5. §3.2 gives the same layout as the 63-byte WebHID buffer, where the header is bytes 0..4.

**Normative numbers for implementers:**

- Full USB report length: **64** bytes (`reportId` + 63 bytes).
- Header after `reportId`: **5** bytes (`flags` … `payloadLen`).
- Max chunk: **58** bytes.
- Max reassembled JSON: **2048** bytes (`caps.maxJsonBytes`). The device sends replies of up to 2047 bytes (36 fragments). It accepts requests with `fragCount ≤ 35`, i.e. at most 2030 bytes.
- Reassembly timeout: **500 ms** from first fragment; incomplete messages are discarded.
- An invalid or oversized inbound request is **dropped silently**: the device sends no reply, because it cannot trust the `id`. See §3.4.

### 3.2 WebHID `DataView` note

`HIDInputReportEvent.data` is a `DataView` of the report **without** the report ID in Chromium (the ID is in `event.reportId`). Host TX via `device.sendReport(reportId, buffer)` also omits the ID from `buffer`.

Therefore browser code should treat the **63-byte** buffer as:

| Offset | Field |
|---:|---|
| 0 | `flags` |
| 1 | `msgSeq` |
| 2 | `fragIndex` |
| 3 | `fragCount` |
| 4 | `payloadLen` |
| 5.. | UTF-8 chunk (`payloadLen` ≤ 58) |

Firmware descriptors still include report ID `0x01` as the first byte of the on-wire report.

### 3.3 Fragment rules

- `fragCount == 1` ⇒ single-report message; `moreFragments` clear; `fragIndex == 0`.
- Multi-fragment: `fragIndex` runs `0 .. fragCount-1`; bit0 of `flags` set on all but the last fragment.
- Receiver keys reassembly by `(direction, msgSeq)`. A new `msgSeq` abandons any incomplete prior message on that path.
- Host and device each maintain their own outbound `msgSeq` counter.

### 3.4 Reassembly corner cases (device receiver, `hid_control.cpp`)

The device keeps **one** reassembly buffer per link. What it does with unusual input:

| Input | Device behaviour |
|---|---|
| Fragment with a new `msgSeq` while a message is incomplete | The partial message is discarded and reassembly restarts with this fragment. |
| Duplicate `fragIndex` within the current message | Ignored (the first copy is kept). |
| **Different `fragCount` on the same `msgSeq`** | The partial message is discarded **and the offending fragment is dropped**. Fragments that follow with that `msgSeq` start a new reassembly with their own `fragCount`, which cannot complete without the lost fragments, so it times out. Treat the message as lost and resend it under a fresh `msgSeq`. |
| **Duplicate `msgSeq` after the message completed** | Once a message is complete and dispatched the receiver forgets it, so re-sent fragments with the same `msgSeq` are accepted as a **new** message and executed again. Never resend the fragments of a completed request; use a new `msgSeq` for every message (the 8-bit counter wraps). This matters for non-idempotent ops such as `setDevice`, `start`, `stop`, `saveTimebase`, `clearTimebaseLog`. |
| New request while the previous complete request has not been handled yet | Dropped (one request at a time). Keep one request in flight and wait for its reply. |
| `fragCount == 0`, `fragIndex ≥ fragCount`, `payloadLen > 58`, `fragCount > 35` (more than 2030 bytes), or a report shorter than its header | Dropped, reassembly state cleared. |
| No further fragment within 500 ms of the first | Partial message discarded. |

**No `tooLarge` for an oversized request.** The device cannot reply to a message it never reassembled: when the first fragment declares an impossible `fragCount` (or is otherwise dropped) no reply of any kind is sent, so a host that sends such a request, or loses a first fragment, may never see a `tooLarge` error. Hosts must rely on their own request timeout (the reference page uses 2-4 s) and must not send requests above 2030 bytes. The error code `tooLarge` is only sent when a **reply** would exceed 2048 bytes (§4.5).

The same rules are a good model for the host's receiver: key by `msgSeq`, restart on a new `msgSeq`, ignore duplicate `fragIndex`, and discard on timeout. Device replies and notifies share the one device-side `msgSeq` counter of their link, and a notify is only started while no other transfer is in flight, so messages never interleave.

---

## 4. JSON message envelope

Protocol version field: `"v": 1`. Bump only on breaking changes. Unknown `op` → `error.code = "unknownOp"`.

Additive changes (new ops, new keys in a result, new `subscribe` fields, new notifies) keep `v` and `protocolVersion` at 1; hosts must ignore keys they do not know. Feature presence is advertised in `hello.caps` instead (e.g. `caps.quartz`). The quartz additions (§5.3, §8.5, §8.6), the H6 additions (§5.4 to §5.9, §8.7) and the H7 additions (§4.5, §4.6, §5.1, §5.10, §7.2) are such changes.

### 4.1 Request (host → device)

```json
{ "v": 1, "id": 7, "op": "getState" }
```

Optional body fields sit alongside `op` (e.g. `patch` for `patchState`, `limit` for `getEvents`).

**How the device reads a request (lenient scanner, not a JSON parser).** The firmware looks keys up by text (`"key"` followed by `:`), the first occurrence wins, and it does not validate the rest of the document. Consequences for hosts:

- Send flat, well-formed JSON with the envelope keys (`v`, `id`, `op`) first, and never put a string value that looks like a key (e.g. `"id"`) in a field.
- `v` must be the integer `1`, `id` an integer. `id` (and `ping.t`) are parsed and echoed as full **64-bit signed** integers, so `Date.now()` round-trips; the reply `id` is the request `id` verbatim (a value outside the int64 range is treated as missing). Keep ids below 2^53 so a JavaScript number holds them exactly.
- Integer fields (`t`, `limit`, `max`, `offset`, the `subscribe` rates) are truncated toward zero if fractional and clamped as described per op. `ping.t` is echoed as a 64-bit signed integer, so a JavaScript `Date.now()` (about 1.7e12) comes back unchanged. The other integer fields are 32-bit quantities and saturate to the 32-bit range before the per-op clamps.
- A field of the wrong JSON type (a string where a number is expected, a number where a boolean is expected) is treated as **absent**, not as an error. Booleans must be the literals `true` / `false`.
- Unknown keys are ignored.

### 4.2 Success response

```json
{ "v": 1, "id": 7, "ok": true, "result": { } }
```

### 4.3 Error response

```json
{
  "v": 1,
  "id": 7,
  "ok": false,
  "error": { "code": "badParam", "message": "quartz.kSigma must be 4..20" }
}
```

### 4.4 Notification (device → host, unsolicited)

```json
{
  "v": 1,
  "notify": true,
  "op": "metrics",
  "data": { "prePeak": 0.12, "agcGainDb": 0 }
}
```

Notifications **omit** `id`. Do not treat them as replies to in-flight requests.

### 4.5 Error codes

| `code` | Meaning |
|---|---|
| `unknownOp` | `op` not recognized (`message` is the op name) |
| `badRequest` | Missing/invalid envelope fields, or an op the current state does not allow (`quartzRestart` in mechanical mode or on a build without quartz; `getGps` without GPS; `start` on an unhealthy backend; `setTimebase` with neither `ppm` nor `logDeltaPpm`) |
| `badParam` | Param name/value rejected by the quartz engine or an op (`message` names the field and its range, e.g. `quartz.kSigma must be 4..20`). A rejected **block** param or enable in `patchState` replies the fixed text `patch rejected` (the message does not name the field; check the range tables in §6.3) |
| `notFound` | Reserved (unknown block name); the current firmware does not send it |
| `tooLarge` | A **reply** would exceed 2048 bytes (`"response exceeds buffer"`). Oversized *requests* are dropped silently (§3.4) |
| `busy` | A flash write was refused because the USB audio stream is open: `saveTimebase`, `setTimebase` with a changed `logDeltaPpm`, `clearTimebaseLog`, `setDevice` |
| `flashError` | The flash write or read-back failed (`"flash write failed"`) |
| `linkBusy` | H7. The request came from a link that does not own control while another link holds it (§4.6). The request was **not** run. `message` names the owner: `control held by usbHid` or `control held by ble` |
| `internal` | Reserved; the current firmware does not send it |

Exact `message` strings of the error replies the firmware emits (a host may show them, never parse them):

| `code` | `message` | When |
|---|---|---|
| `badRequest` | `missing id` / `unsupported protocol version` / `missing op` / `missing patch object` | Envelope (`id` is 0 in the first case; `missing patch object` is `patchState` without a `patch` object) |
| `badParam` | `patch rejected` | `patchState`: `blocks` or a block entry is not an object, a block param is out of range, or an enable is refused (fields applied earlier in the patch stay applied). Unknown block names, unknown param names and read-only params are silently skipped |
| `badParam` | `quartz must be an object` | `patchState` with a non-object `quartz` |
| `badParam` | `quartz.mode must be mechanical, motor, lcd or fork` / `quartz.mode must be a string` / `quartz.tsMethod must be threshold or xcorr` | `patchState.quartz` |
| `badParam` | `quartz.gateS must be 0 or 3..960` / `quartz.forkHz must be 300, 341.33, 360, 480 or 720` / `quartz.kSigma must be 4..20` | `patchState.quartz` numeric settings |
| `badParam` | `quartz timing needs the PCM1808 source` | `patchState.quartz` anything but `mode: "mechanical"` on a build without the quartz engine (`caps.quartz` false) |
| `badRequest` | `quartzRestart needs motor, lcd or fork mode` / `quartz timing needs the PCM1808 source` | `quartzRestart` in mechanical mode / on a build without quartz |
| `unknownOp` | the op name | Unknown `op` |
| `badRequest` | `capture backend error (stream state: error)` | `start` while the capture backend is unhealthy |
| `badRequest` | `GPS not built in` | `getGps` when `caps.gps` is `false` |
| `badRequest` | `missing ppm` | `setTimebase` with neither `ppm` nor `logDeltaPpm` |
| `badParam` | `device.mode must be usbMic or standalone` | `setDevice` with a missing or unknown `mode` |
| `badParam` | `logDeltaPpm must be 0.05..10` | `setTimebase` |
| `badParam` | `source must be none, manual or ntp` | `setTimebase` |
| `badParam` | `ppm must be -1000..1000` | `setTimebase` (also for a `dateUnix` / `durationS` outside 0..4294967295) |
| `busy` | `close the audio stream before changing mode` | `setDevice` while a USB stream is open |
| `busy` | `close the audio stream before saving` | `saveTimebase`, `setTimebase.logDeltaPpm`, `clearTimebaseLog` while a USB stream is open |
| `flashError` | `flash write failed` | `setDevice`, `saveTimebase`, `setTimebase.logDeltaPpm`, `clearTimebaseLog` |
| `linkBusy` | `control held by usbHid` / `control held by ble` | §4.6: a non-owner link sent an op (other than `hello`, `ping`, `claim`) while another link holds control |
| `tooLarge` | `response exceeds buffer` | Any reply of 2048 bytes or more (the buffer holds 2047 characters). Most results are far below that; none of the list ops is an exception any more: `getEvents` and `getBeats` drop what does not fit (§5, §8.3) |

### 4.6 Control ownership

H7. With two links (USB HID and BLE, [`BLE_CONTROL_PLANE.md`](BLE_CONTROL_PLANE.md)), at most one link **owns** the control plane. The owner's requests reach the device's handler and its subscriptions; the other link gets only the answers that do not change state. A build with only USB HID (`TG_BLE` off) behaves as before: its one link is always the owner.

**Rule:** at most one link **owns** the control plane. State is `owner ∈ {none, usbHid, ble}`, plus `lease_deadline`. `kLeaseMs = 10000`.

| Request arriving on link L | Behaviour |
|---|---|
| `hello`, `ping` | Always answered. Never claims, never refreshes another link's lease. A `ping` **from the owner** refreshes its lease. `hello.control` reports the state. |
| `claim` with no `takeover` (or `false`) | If owner is none, L, or the lease has expired: L becomes owner and the lease is refreshed. Otherwise: `linkBusy`. |
| `claim` with `takeover: true` | L becomes owner unconditionally. If the previous owner P ≠ L: `ControlHandler::reset()` (P's subscriptions stop), and P gets `controlReleased` if its link is still connected. |
| Any other op | If owner is none, L, or the lease has expired: **auto-claim** (L becomes owner, lease refreshed), then dispatch to `ControlHandler`. If ownership moved from another link P by expiry, the handler is reset first (and P gets `controlReleased`). Otherwise: reply `linkBusy` and don't dispatch. |

| Event | Behaviour |
|---|---|
| Owner link goes down (BLE disconnect or CCCD off; USB unmount) | owner = none; `ControlHandler::reset()`. |
| Lease expires (no request from the owner for 10 s) | Nothing happens by itself: subscriptions keep running. The **next** claim or auto-claim by the other link takes over (as above). |
| Pushes | `ControlHandler::poll()` output goes **only** to the owner's session. |

Notes:

- Auto-claim keeps old hosts (`example.html`, older web app builds, scripts) working unchanged when they are the only client, and a non-W build (only `usbHid`) behaves exactly as today.
- Hosts that stay connected without sending requests (the web app on a passive view) must keep the lease alive: the web transports send `ping` after 3 s without any request. WebHID gives the device no close signal, so without the lease a closed desktop tab would block BLE forever.
- `hello.control` tells a host who owns control before it sends anything else (§5.1). A host that finds another owner should ask the user before it claims with `takeover: true`.

---

## 5. Operations

Most ops map through `control_api`. CLI string commands (`dsp set …`) are **not** exposed on HID. The link and ownership ops (`claim`, `getLink`, `getDiag`) are described in §5.10.

| `op` | Body fields | `result` |
|---|---|---|
| `hello` | — | Device identity + caps (§5.1) |
| `ping` | optional `t` (integer, echoed as a 64-bit signed value, default 0; see §4.1) | `{ "t": <echo>, "uptimeMs": N }` |
| `getState` | — | Full `DspState` (§6) |
| `patchState` | `patch`: partial `DspState` | Post-patch full `DspState` |
| `getMetrics` | — | `DspMetrics` |
| `getTiming` | — | `DspTiming` (DSP block + main-loop CPU headroom; `quartzBlockUs*` / `quartzBlocks` = quartz engine cost per capture block, 0 without PCM1808) |
| `resetTiming` | — | `{ "cleared": true }` |
| `getBeat` | — | `DspBeatRate` |
| `getSpectrum` | — | `{ "valid", "fftN", "sourceHz", "nPeaks", "peaks": [{ "hz", "mag" }] }` (`DspSpectrum`): spectral peaks of the analysis source (`spectrum` block) |
| `getEvents` | optional `limit` (clamped to 1..32, default 8) | `{ "events": DspEvent[] }` newest first. An event is 216 to 345 bytes, so the reply holds as many of the requested events as fit one 2048-byte message, always the **newest** ones (a prefix of the newest-first list): with large values it may carry fewer than `limit` (at least 5 even at the worst case), never more, and never `tooLarge`. There is no offset parameter; events beyond the reply are reachable through the `event` notify |
| `getErrors` | — | `{ "errors": DspError[] }` newest first |
| `clearErrors` | — | `{ "cleared": true }` |
| `subscribe` | rates/flags (§7); no field is required, none is ever an error (rates are clamped) | Echo of the full active subscription |
| `getTime` | — | `{ "tUs": N, "uptimeMs": N }`, the device µs timer, sampled first (calibration probe) |
| `getTimebase` | — | `Timebase` (§5.2) |
| `setTimebase` | `ppm` (−1000..1000), optional `source` (`"none"` / `"manual"` / `"ntp"`, default `"manual"`), `dateUnix`, `durationS` (each 0..4294967295, default 0); and/or `logDeltaPpm` (0.05..10). `ppm` is optional when `logDeltaPpm` is present (§5.9) | `Timebase`; the calibration is RAM only (works while audio streams), `logDeltaPpm` is written to flash (`busy` while streaming when it changes) |
| `saveTimebase` | — | `Timebase`; writes flash and appends a `save` entry to the timebase log (§5.8). `busy` while streaming |
| `clearTimebase` | — | `Timebase`; 0 ppm / `"none"` in RAM |
| `getQuartz` | — | `DspQuartz` (§8.5): quartz-watch timing snapshot |
| `getQuartzPulses` | optional `max` (1..8, default 8) | `{ "pulses": DspQuartzPulse[] }` (§8.6), newest first; empty outside motor mode |
| `quartzRestart` | — | `DspQuartz` after starting a new measurement in the active mode (also leaves `done`). `badRequest` in mechanical mode or without quartz |
| `getDevice` | — | `DeviceStatus` (§5.5) |
| `setDevice` | `mode`: `"usbMic"` \| `"standalone"` | `DeviceStatus`; persists the mode to flash. `badParam`; `busy` while a USB stream is open |
| `start` | — | `DeviceStatus` after setting the run latch (§5.4). `badRequest` if the backend is unhealthy |
| `stop` | — | `DeviceStatus` after clearing the run latch. Never fails |
| `getBeats` | optional `max` (1..`caps.beatsMax` = 8, default 8), optional `packed` (boolean) | `{ "beats": DspBeat[] }` (§8.7), newest first; with `packed: true` `{ "p": "<base64>" }` (§5.6.1), `max` up to 32 |
| `getGps` | — | `GpsInfo` (§5.7). `badRequest` without GPS |
| `getTimebaseLog` | optional `offset` (integer, default 0; negative is 0; past the end gives an empty page) | `TimebaseLog` page (§5.8) |
| `clearTimebaseLog` | — | `TimebaseLog` (empty, offset 0); writes flash. `busy` while streaming |
| `claim` | optional `takeover` (boolean, default false) | `{ "owner", "previous" }` (§5.10). Takes control for the requesting link. Without `takeover`, `linkBusy` while another link holds a live lease. H7 |
| `getLink` | — | `{ "usbHid": { "mounted" }, "ble"?: { … }, "control": { … } }` (§5.10). Owner-only under §4.6: a non-owner gets `linkBusy` while another link holds a live lease; otherwise the request auto-claims control. Read `hello.control` to see the owner without claiming. H7 |
| `getDiag` | — | The diagnostic counters, camelCase (§5.10). Same ownership rule as `getLink` (§4.6): `linkBusy` for a non-owner while another link holds a live lease, otherwise auto-claim. H7 |

Quartz settings (mode, gate, fork frequency, timestamp method, threshold) are part of `getState` / `patchState` (§6), not separate ops. Device mode and run control (`getDevice`, `setDevice`, `start`, `stop`) are described in §5.4 and §5.5.

### 5.1 `hello` result

```json
{
  "protocolVersion": 1,
  "fwVersion": "0.1.0-dev",
  "board": "Pico2",
  "source": "PCM1808",
  "vid": 51966,
  "pid": 16387,
  "caps": {
    "maxJsonBytes": 2048,
    "metricsHzMax": 20,
    "beatHzMax": 5,
    "timingHzMax": 5,
    "timing": true,
    "quartz": true,
    "quartzHzMax": 5,
    "quartzPulsesMax": 8,
    "deviceMode": true,
    "beats": true,
    "beatsMax": 8,
    "beatsPacked": 1,
    "gps": false,
    "pps": true,
    "timebaseLog": true,
    "timebaseLogMax": 24,
    "blocks": [
      "levelMeter", "gain", "limiter", "filterBank",
      "eventDetect", "beatRate", "noiseFloor", "eventLog",
      "softGate", "notch", "agc", "firBank", "spectrum"
    ],
    "pipeline": [
      { "key": "input", "group": "source", "order": 0, "inputs": [] },
      { "key": "levelMeter", "group": "analysis", "order": 0, "inputs": ["input", "limiter"] },
      "…"
    ]
  }
}
```

The `pipeline` array is abbreviated here; the full node list is in the table below.

`board` is `"Pico2"`, `"Pico2W"` (W builds, `TG_BOARD_NAME`), `"KiwiBoard"` or `"RP2350"`; `source` is `"PCM1808"`, `"ADC"` or `"SYNTHETIC"`; `fwVersion` is the build's `FW_VERSION`. `caps.timing` is always `true`. The example above is the `pico2_pcm1808` build. Build-dependent caps: `caps.pps` is `true` only when the build sets `PPS_GPIO` (`pico2_pcm1808` does, with `-DPPS_GPIO=3`); `caps.gps` is `true` only with `GPS_UART_RX` (no shipped env sets it, so it is `false`); `caps.quartz` and the `quartz` pipeline node need the PCM1808 source; `board`, `source`, `vid`/`pid` and `fwVersion` follow the build.

`caps.blocks` lists JSON camelCase keys available in `getState` / `patchState`. `caps.timing` / `timingHzMax` advertise CPU headroom telemetry (`getTiming` / `subscribe.timingHz`).

Capability keys added by H6 (additive; an **absent key means the feature is absent**, so a host talking to older firmware must treat each as `false`):

| Key | Type | Meaning |
|---|---|---|
| `deviceMode` | bool | `getDevice` / `setDevice` / `start` / `stop` exist (§5.4, §5.5). Current firmware always sends `true` |
| `beats` | bool | `getBeats` and `subscribe.beats` / the `beats` notify exist (§5.6). Current firmware sends `true` |
| `beatsMax` | number | Largest `getBeats.max`: **8** (the notify also carries at most 8 beats) |
| `beatsPacked` | number | Version of the packed beats format (§5.6.1): `1`. Absent = `subscribe.beatsPacked` and `getBeats.packed` do not exist |
| `gps` | bool | `getGps` exists **and** the build has `GPS_UART_RX`. When `false`, `getGps` replies `badRequest` `GPS not built in` |
| `pps` | bool | The build has `PPS_GPIO` (`Timebase.pps.enabled`, the OLED GPS badge) |
| `timebaseLog` | bool | `getTimebaseLog` / `clearTimebaseLog` exist (§5.8). Current firmware sends `true` |
| `timebaseLogMax` | number | Timebase-log ring size: **24** |
| `pipeline` | `PipelineNode[]` | The DSP data-flow graph (below) |

Capability keys added by H7 (same rule: absent means `false`):

| Key | Type | Meaning |
|---|---|---|
| `ble` | bool | The firmware is built with BLE (`TG_BLE=1`, Pico 2 W). The `getLink.ble` object exists (§5.10) |
| `cli` | bool | The serial CLI is built (`TG_SERIAL_CLI=1`, see [`build_and_flash.md`](build_and_flash.md)). Without it the CDC port carries no CLI |

**`hello.link` and `hello.control` (H7).** `hello` also reports the link the request arrived on and who owns control. Both are absent on older firmware.

```json
"link": { "kind": "ble", "mtu": 185, "frameMax": 63 },
"control": { "owner": "ble", "self": true, "leaseMs": 10000 }
```

- `link.kind`: `"usbHid"` or `"ble"`, the link this request arrived on. `link.mtu` is present only for `ble`. `link.frameMax` is the device-to-host frame size in bytes for messages after this reply: 63, or the size a BLE host chose with `hello {"frameMax": N}` ([`BLE_CONTROL_PLANE.md`](BLE_CONTROL_PLANE.md) §4.3a). USB HID always reports 63.
- `control.owner`: `"usbHid"`, `"ble"` or `null` (no owner). `control.self` is `true` when the owner is the link this request arrived on. `control.leaseMs` is the lease length (10000). Rules: §4.6.

**`caps.pipeline`.** Describes how data flows through the DSP so that a UI can draw it from the device instead of hard-coding it. Each node is

```json
{ "key": "eventDetect", "group": "analysis", "order": 4, "inputs": ["input", "noiseFloor"] }
```

- `key`: a block key from `caps.blocks`, or one of the pseudo-nodes `input`, `pcm24`, `usb`, `quartz`. `bypass` is not a node (it gates the whole process chain).
- `group` (the node kind): `source` (`input`, the raw capture), `analysis` (read-only taps of `input`, they never change the audio), `process` (the in-place process chain), `sink` (`pcm24`, `usb`), `quartz` (the quartz engine).
- `order`: position in the real processing sequence **within its group** (analysis taps run in `order`; the process chain runs in `order`; the sinks are 8 and 9, the others restart at 0).
- `inputs`: keys of the nodes this node reads. The graph is acyclic.

As built (from `src/dsp_blocks.cpp`, derived from `dsp_graph_process`):

| key | group | order | inputs |
|---|---|---:|---|
| `input` | source | 0 | none |
| `levelMeter` | analysis | 0 | `input`, `limiter` (measures pre on `input` and post at the end of the chain; the post tap only reads) |
| `spectrum` | analysis | 2 | `input` |
| `noiseFloor` | analysis | 3 | `input` |
| `eventDetect` | analysis | 4 | `input`, `noiseFloor` |
| `eventLog` | analysis | 5 | `eventDetect` |
| `beatRate` | analysis | 6 | `eventDetect` (a sibling of `eventLog`: `beatRate` does **not** read `eventLog`) |
| `notch` | process | 1 | `input` |
| `filterBank` | process | 2 | `notch` |
| `firBank` | process | 3 | `filterBank` |
| `softGate` | process | 4 | `firBank` |
| `gain` | process | 5 | `softGate` |
| `agc` | process | 6 | `gain` |
| `limiter` | process | 7 | `agc` |
| `pcm24` | sink | 8 | `limiter` |
| `usb` | sink | 9 | `pcm24` |
| `quartz` | quartz | 0 | `input` |

The `quartz` node (and any input that names it) is **omitted when `caps.quartz` is `false`**. The process chain is skipped while master `bypass` is on; analysis blocks run only when enabled (`getState`). In standalone mode the `usb` sink carries silence (§5.4).

`caps.quartz` is `true` on builds that run the quartz engine (PCM1808 source) and `false` elsewhere (ADC, SYNTHETIC), where only mechanical mode exists: `getState` then has no `quartz` object, `patchState` accepts only `{"quartz":{"mode":"mechanical"}}`, `getQuartz` reports mode `mechanical` / state `idle` with a reason, `getQuartzPulses` is empty, and `subscribe` forces `quartzHz` to 0 and `quartzPulses` to `false`. `quartzHzMax` caps `subscribe.quartzHz`; `quartzPulsesMax` caps `getQuartzPulses.max`.

### 5.2 `Timebase`

Sample-clock calibration (ppm, + = board clock fast). It applies to every timing result. See [TIMEBASE_CALIBRATION.md](TIMEBASE_CALIBRATION.md).

```json
{
  "ppm": -23.412, "source": "ntp",
  "calPpm": -23.412, "calSource": "ntp", "dateUnix": 1763000000, "durationS": 1800,
  "saved": true,
  "logDeltaPpm": 0.2, "logPending": false,
  "pps": { "enabled": false, "locked": false, "ppm": 0, "window": 0, "good": 0, "rejects": 0, "noFix": 0 },
  "gps": { "enabled": false, "fix": false, "quality": 0, "sats": 0, "hdop": 0, "utcUnix": 0,
           "ageMs": -1, "sentences": 0, "errors": 0, "overruns": 0 }
}
```

`ppm` / `source` is the correction in effect: PPS while it is locked, otherwise the stored `cal*` values. `gps` is the NMEA status in builds with `-DGPS_UART_RX=N` (PPS then only counts while `gps.fix` is true; `pps.noFix` counts discarded pulses); `ageMs` is the time since the last RMC sentence, -1 = never. `saved` is true when flash holds exactly the stored values.

`logDeltaPpm` is the persisted threshold (ppm, 0.05..10, default 0.2) that makes a GPS PPS measurement worth appending to the calibration history log; `logPending` is `true` while a log entry has been appended in RAM but its flash write is deferred (USB audio stream open, or a quartz measurement running). See §5.8 and [TIMEBASE_CALIBRATION.md](TIMEBASE_CALIBRATION.md). These two keys are additive: older firmware omits them.

### 5.3 Quartz timing

Quartz-watch timing (handoff `docs/handoff/FIRMWARE_HANDOFF.md`) runs beside the mechanical beat path. One mode is active at a time:

| `mode` | Watch | Channel | Method | Headline `rateSDay` |
|---|---|---|---|---|
| `mechanical` | Mechanical (default) | — | `getBeat` | quartz idle; use `getBeat` |
| `motor` | Analog quartz (stepper motor) | LEFT, inductive | Pulse leading edges, segmented regression | Rate **with** inhibition (what the wearer sees) |
| `lcd` | LCD digital | RIGHT, capacitive | Harmonic phase slope | Oscillator rate (the LCD cannot see inhibition) |
| `fork` | Tuning fork (Accutron and similar) | LEFT | Tone phase slope | Fork rate |

Typical flow:

1. `hello` → check `caps.quartz`.
2. `patchState` `{"quartz":{"mode":"motor","gateS":60}}` → the post-patch `getState` shows the new settings. Selecting a different mode, or changing a setting the active mode uses, starts a new measurement.
3. `subscribe` `{"quartzHz":1,"quartzPulses":true}` → `quartz` notifies (the `getQuartz` snapshot) and one `quartzPulse` notify per motor pulse.
4. When `state` is `done` (motor gate reached), the result is frozen; `quartzRestart` measures again.

`state` values: `idle` (mechanical mode, or no engine), `waitTimebase` (capture rate not verified yet), `unsupportedRate` (verified rate is not 48 kHz), `acquiring` (running, no rate yet; `reason` says what it waits for), `measuring` (a rate exists and is refined), `done` (motor gate reached; result frozen). `reason` is human-readable display text, never parse it. In `usbMic` mode the engine is off (§5.4): `analysis` is `"off"`, `state` is `idle` and `reason` is `"USB Audio mode"`, whatever `mode` is; the mode setting is kept and measuring starts again, from scratch, when `standalone` is selected.

Motor gate (`gateS`): `0` = run until two inhibition events have been seen, otherwise 3..960 s (10 and 60 are the usual choices). A 10 s gate on a watch that inhibits every 60 s reports the raw rate, so always show the inhibition status (`motor.inhibition`, §8.5).

### 5.4 Device modes

The stand works in one of two modalities. The mode is a persisted setting; the **run latch** decides whether capture and analysis run without a USB audio stream.

| `mode` | What it is | Capture + analysis run |
|---|---|---|
| `usbMic` (default) | **USB microphone.** A PC timegrapher program records the audio over UAC2 and does the analysis; the device's own timing engines are off and only the beat rate is detected (see "Timing in each mode" below). | While the host has the audio stream open (`usbStreaming`), **or** while the run latch is set. |
| `standalone` | **Standalone timegrapher.** The device is the timegrapher: it captures and analyses by itself and shows the result on its OLED, CLI and HID. | The run latch is set **at boot and whenever this mode is selected**; no host stream is needed. |

Capture runs while `usbStreaming || runRequested`.

**The run latch (`runRequested`).** A RAM flag, not persisted.
- `start` sets it (a no-op if already set). `stop` clears it. `setDevice` to `standalone` sets it; `setDevice` to `usbMic` clears it. At boot it is set when the persisted mode is `standalone`.
- With the latch set and no USB stream, the device warms up the preamp (if any), starts capture and runs the DSP pipeline exactly as for a stream, so the meters, spectrum, events and the beat-rate lock work with no host audio. In `standalone` mode the timing engines run as well (`getBeat`, `getBeats`, `getQuartz`); in `usbMic` mode they stay off (below).
- If the capture backend is unhealthy (`streamState` `error`), `start` replies `badRequest` and a latch found set by the firmware is dropped again.
- When the host opens the audio stream in `usbMic` mode the stream takes over the pipeline from a latch-run (a fresh stream: the DSP state, the beat `seq` and any quartz measurement restart). If the USB bus is suspended while the latch is set, the latch keeps the device measuring.

**Timing in each mode.** Only `standalone` runs the timing engines. In `usbMic` the host does the timing (from the audio stream), so the device turns its own off and keeps what a status display needs:

| | `standalone` | `usbMic` |
|---|---|---|
| Tick detection, A/h lock (`locked`, `vphNearest`, `vphRaw`, `confidence`, `intervalSamples`) | runs | runs |
| Rate and beat-error fit, rolling means, amplitude, beat error windows, drop timing (`timingLandmark`), no-watch rule | runs | **off** |
| Per-beat records (`getBeats`, `beats` notify) | produced | **none** (they need the fit) |
| Quartz engine (`getQuartz`, `quartz` notify) | runs (when `quartz.mode` is not `mechanical`) | **off**; the `quartz.mode` setting is kept |
| `getBeat.analysis`, `getQuartz.analysis` | `"on"` | `"off"` |
| `getBeat.state` | listening, locking, locked, noWatch | `listening`, `locking` (ticks seen, no lock) or `locked` (lock held); never `noWatch` |
| Audio path (filters, gain for the USB stream) | unchanged | unchanged |

A host that sees `analysis: "off"` should say that timing runs on the host instead of showing the rate, beat error and amplitude (they are invalid). Switching the mode in either direction restarts the engines from a clean state: the lock, the fit and its templates, the beat `seq` and any quartz measurement are cleared, so nothing measured in one mode appears in the other. The block parameters (`liftAngle`, `timingLandmark`, ...) and the quartz settings are kept.

**`start` / `stop` per mode.**

| Situation | `start` | `stop` |
|---|---|---|
| `usbMic`, no stream | Sets the latch: capture runs without a stream | Clears the latch: capture stops |
| `usbMic`, stream open | No visible change (capture already runs for the stream) | Clears the latch. **`running` stays `true`** while the stream is open: `stop` cannot stop a recording the host started |
| `standalone` | Sets the latch (after an earlier `stop`) | Clears the latch: capture stops and the device **stays stopped** until `start` or the next boot (selecting `standalone` again is a no-op while it is already selected) |

A reply to `stop` is built immediately; the capture backend stops on the next main-loop pass, so `running` in the `stop` reply can still read `true` for a moment even with no stream. Poll `getDevice` to see the settled state. `running` is also `false` while the preamp warms up.

**Persistence.** `setDevice` writes the mode to flash (the DEVICE region, together with the timebase-log threshold `logDeltaPpm`; see [AS_BUILT_ARCHITECTURE.md](AS_BUILT_ARCHITECTURE.md) "Device mode and flash store"). A flash erase stalls the CPU for tens of ms, so the write is refused (`busy`, `close the audio stream before changing mode`) while the USB audio stream is open; otherwise capture is stopped around the write and re-armed afterwards (a fresh stream: `seq` of `beats` restarts at 0, a quartz measurement restarts). Setting the mode that is already active is a no-op: it writes nothing, never replies `busy` (even while a USB stream is open) and does not touch the run latch. `DeviceStatus.saved` is `false` until flash holds the current mode (a never-configured device runs `usbMic` with `saved: false`).

**Standalone and the USB audio feed.** In `standalone` mode the capture backends do not push to the PCM ring and skip the 24 to 48 kHz repeat; if a host nevertheless opens the audio stream, the USB IN packets are **zero-filled** (silence, not an underrun) and `usbStreaming` is still reported `true`. A standalone device therefore never makes a sound recording; it only reports results.

**Display.** The OLED header shows `SA` (standalone) or `USB` (see [DISPLAY.md](DISPLAY.md)). CLI: `device` / `device mode usbmic|standalone`, `run start|stop|status`.

### 5.5 `getDevice` / `setDevice` / `start` / `stop`

All four reply with `DeviceStatus`:

```json
{ "mode": "standalone", "running": true, "runRequested": true,
  "usbStreaming": false, "streamState": "idle", "saved": true }
```

| Key | Type | Meaning |
|---|---|---|
| `mode` | `"usbMic"` \| `"standalone"` | Current mode |
| `running` | bool | Capture + analysis are running (for any reason), after any preamp warm-up |
| `runRequested` | bool | The run latch (set by `start`, cleared by `stop`; set at boot in standalone) |
| `usbStreaming` | bool | The USB host has the audio stream open |
| `streamState` | string | `unconfigured` (not mounted) \| `idle` (enumerated, no stream) \| `prefill` \| `active` \| `suspended` (bus suspend) \| `error` (capture backend failed to initialise) |
| `saved` | bool | Flash holds the current mode |

Requests:

```json
{ "v": 1, "id": 31, "op": "setDevice", "mode": "standalone" }
{ "v": 1, "id": 32, "op": "start" }
```

`setDevice` mode names are matched case-insensitively; besides `usbMic` / `standalone` the firmware also takes `usb_mic`, `usb` and `sa`, but hosts should send the canonical names. Errors: `badParam` `device.mode must be usbMic or standalone` (missing or unknown `mode`); `busy` `close the audio stream before changing mode`; `flashError` `flash write failed` (the previous mode is kept); `start`: `badRequest` `capture backend error (stream state: error)`. `getDevice` and `stop` never fail.

### 5.6 `getBeats` and the `beats` stream

`getBeats { "max": 8 }` returns the newest beats, **newest first**: `{ "beats": DspBeat[] }` (§8.7). `max` is clamped to 1..`caps.beatsMax` (8). The device keeps the last **64** beats in a ring (`dsp_beat_t`, `DSP_BEAT_LOG_CAP`). A beat object is up to 288 bytes, so the reply holds as many of the requested beats as fit one 2048-byte message: with extreme values it may carry fewer than `max`, never more. One beat record exists per tick handed to the beat fit once the A/h lock is established; none while unlocked, and none in `usbMic` mode (`getBeat.analysis` `"off"`, §5.4).

`subscribe { "beats": true }` pushes `beats` notifies:

```json
{ "v": 1, "notify": true, "op": "beats", "data": { "beats": [ /* DspBeat, oldest first */ ] } }
```

Push order and baseline rules (firmware: `push_notify_beats`):
- **Oldest first**, at most **8** beats per notify. A notify that would not fit one message carries fewer; the rest follow in the next notify.
- The cursor is the beat `seq`. **`subscribe` seeds it** with the sequence the next beat will receive, so subscribing never replays history; only beats that arrive after the subscribe are pushed (a `getBeats` call gives the history).
- If the 64-entry ring wrapped past the cursor (the host stalled for more than 64 beats) the lost beats are skipped, never repeated.
- **Stream reset:** on a stream reset (capture restart, `setDevice`, a flash commit, a USB stream open or close) the numbering restarts at 0 and the ring is cleared. The device detects this when the beat `seq` is **below** its cursor and then **re-seeds the cursor to 0**, so every beat since the reset is pushed, starting with `seq` 0. Hosts must treat a `seq` that goes backwards, or a `beats` notify whose first `seq` is 0, as the start of a new stream: drop the old plot. (If so many beats arrive between two polls that `seq` overtakes the old cursor, the reset cannot be seen from `seq` alone; also watch `gridId` and `sampleIndex`, which restart.)
- Competing pushes are sent one at a time in the order metrics, beat, timing, quartz, quartzPulse, **beats** (or `beatsPacked`), event, error (§7.1).

#### 5.6.1 Packed beats (`beatsPacked`)

A `DspBeat` object costs about 200 bytes of JSON; at 36,000 bph that is more than a BLE link carries. The packed format sends the same beats as a little-endian binary block, base64 encoded (RFC 4648 §4, `=` padded), under one key. The envelope stays JSON.

- **Subscribe.** `subscribe { "beatsPacked": true }` pushes `beatsPacked` notifies (`data: { "p": "..." }`). It replaces the `beats` stream: when both flags are set only `beatsPacked` is sent, and `false` restores `beats` if that is still set. The cursor, baseline and stream-reset rules are those of `beats` (§5.6); the batch holds up to 32 beats.
- **History.** `getBeats { "max": n, "packed": true }` returns `{ "p": "..." }` with `max` clamped to 1..32. The beats are in seq order, newest **last**. If a batch break (below) or the message size splits the requested beats, the reply holds the newest batch only.
- **Capability.** `hello.caps.beatsPacked` is the format version (`1`); absent means not supported.
- **`sourceHz`** for `tS` comes from `getBeat.sourceHz`.

Header (format 1; `i` signed, `u` unsigned):

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `fmt` | 1 |
| 1 | u8 | `n` | Beats in the batch, 1..32 |
| 2 | u32 | `seq0` | `seq` of the first beat |
| 6 | u32 | `gridId` | Grid of every beat in the batch |
| 10 | f32 | `nominalUs` | Nominal half-period, µs |
| 14 | u64 | `sample0` | `sampleIndex` of the first beat |
| 22 | i32 | `k0` | `k` of the first beat |
| 26 | n × 17 B | beats | Below |

Per beat (17 bytes):

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `dSeq` | `seq` minus the previous `seq` (0 for the first beat) |
| 1 | u8 | `flags` | Bit 0 parity, bit 1 `amplitudeValid`, bits 2-3 status (0 accepted, 1 started, 2 rejected) |
| 2 | u32 | `dSample` | `sampleIndex` minus the previous one (0 for the first) |
| 6 | u8 | `frac` | Fractional unlock sample × 256: `tS = (sampleIndex + frac / 256) / sourceHz` |
| 7 | i16 | `dK` | `k` minus the previous `k` (0 for the first) |
| 9 | i32 | `u` | Residual `uUs`, 0.1 µs units |
| 13 | u16 | `amp` | `amplitudeDeg`, 0.01° units (0 when invalid) |
| 15 | u16 | `lift` | Lift time, µs (`liftMs` × 1000; saturates at 65,535) |

Quantisation errors: `u` ±0.05 µs, `amp` ±0.005°, `lift` ±0.5 µs, `frac` ±1/256 sample. The reconstructed `liftMs` is `lift / 1000`.

**Batch breaks.** A batch ends, and the next notify starts a new header with the breaking beat, when the grid or the nominal period changes, when a delta does not fit its field (`dSeq` above 255 or not increasing, `dSample` of 2³² or more, `dK` outside i16), after 32 beats, or when the base64 would not fit the 2048-byte message. A decoder needs no grid-change logic inside a batch.

**Size.** 8 beats are 26 + 8 × 17 = 162 bytes, 216 base64 characters, about 280 bytes with the envelope.

### 5.7 `getGps`

Detailed GPS state (requires `caps.gps`; `badRequest` `GPS not built in` otherwise). The satellites are **compact tuples** so that 32 of them always fit one message; the column order is given in `satCols` and is fixed.

```json
{
  "fix": true, "quality": 1, "utcUnix": 1794746119, "ageMs": 412,
  "satsUsed": 9, "hdop": 0.92, "pdop": 1.5, "vdop": 1.2,
  "lat": 47.6205063, "lon": -122.3490000, "altM": 56.0, "posValid": true,
  "satCols": ["sys", "prn", "el", "az", "snr", "used"],
  "sats": [ ["GP", 12, 45, 123, 38, 1], ["GL", 70, 20, 301, -1, 0] ]
}
```

| Key | Meaning |
|---|---|
| `fix` | Valid fix now (same definition as `Timebase.gps.fix`: RMC `A` and, if GGA is present, quality > 0 with at least `GPS_MIN_SATS` satellites) |
| `quality` | GGA fix quality, 0 = none or stale |
| `utcUnix` | UTC (UNIX s), 0 = never received |
| `ageMs` | ms since the last RMC sentence, -1 = never |
| `satsUsed` | Satellites in use (GGA); 0 if stale |
| `hdop` / `pdop` / `vdop` | Dilution of precision, 0 = unknown (HDOP is GGA, GSA if GGA is stale) |
| `lat` / `lon` / `altM` | Degrees (+N, +E) and metres above MSL. Meaningful only when `posValid`; clamped to the physical range |
| `satCols` | Always `["sys","prn","el","az","snr","used"]` |
| `sats` | Up to **32** tuples, in `satCols` order. Satellites not heard for 5 s are dropped |

Tuple fields: `sys` is `"GP"` (GPS), `"GL"` (GLONASS), `"GA"` (Galileo), `"BD"` (BeiDou), `"QZ"` (QZSS) or `"??"`; `prn` is the normalised PRN; `el` elevation in degrees, `-1` unknown; `az` azimuth in degrees, `-1` unknown; `snr` C/N0 in dB-Hz, `-1` = not tracked; `used` is the **number** `1` when the satellite is in the position solution (GSA), else `0`. NMEA parsing (GSV, GSA, GGA, RMC, any talker, multi-GNSS) is in `include/nmea.h`. CLI: `timebase gps`.

### 5.8 Timebase calibration log: `getTimebaseLog` / `clearTimebaseLog`

The device keeps a history of the clock calibrations in a **24-entry ring in flash** (oldest dropped). Entries are added by two triggers: every successful `saveTimebase` (reason `save`), and a trustworthy GPS PPS measurement that moved enough or is old enough (`delta`, `monthly`). Policy, flash layout and the flash-wear reasoning are in [TIMEBASE_CALIBRATION.md](TIMEBASE_CALIBRATION.md); in short:
- A PPS entry is only a candidate with the PPS window full (64 pulses), PPS **continuously locked for at least 600 s**, and a GPS UTC date (no UTC, no PPS entry).
- It is appended when the log is empty (`delta`), when at least **30 days** have passed since the last entry of any source (`monthly`), or when `|ppm - last.ppm| >= logDeltaPpm` **and** at least **1 day** has passed (`delta`). `logDeltaPpm` is 0.05..10, default 0.2.
- The flash write is **deferred**: never while the USB audio stream is open, nor while a quartz measurement is acquiring or measuring; one attempt per 10 s (60 s after a failure). Until written, `pending` (and `Timebase.logPending`) is `true`.

`getTimebaseLog { "offset": 0 }` replies with one **page**:

```json
{ "entries": [ { "dateUnix": 1794746119, "ppm": -23.412, "source": "ntp", "durationS": 1800, "reason": "save" } ],
  "total": 1, "offset": 0, "pending": false, "logDeltaPpm": 0.2, "lastWriteUnix": 1794746119 }
```

| Key | Meaning |
|---|---|
| `entries` | Log entries `[offset, offset + entries.length)`, **oldest first**, as many as fit one message |
| `total` | Entries in the whole log (0..`caps.timebaseLogMax`) |
| `offset` | First entry of this page: the request's `offset`, clamped to `total` (`offset` at or past `total` gives an empty page; a negative `offset` is 0) |
| `pending` | A log flash write is deferred |
| `logDeltaPpm` | Current append threshold |
| `lastWriteUnix` | GPS UTC of the last successful log write, 0 = never or unknown |

**Paging.** A full ring of 24 needs at most 2 pages: repeat with `offset += entries.length` until `offset + entries.length >= total` (stop on an empty page).

Entry: `dateUnix` (UTC seconds the calibration is dated), `ppm`, `source` (`"manual"` \| `"ntp"` \| `"pps"`), `durationS` (measurement length, 0 = unknown or manual), `reason` (`"save"` \| `"delta"` \| `"monthly"`).

`clearTimebaseLog` empties the log in RAM and flash and replies like `getTimebaseLog` with offset 0 (an empty page). It writes flash: `busy` `close the audio stream before saving` while a USB stream is open, `flashError` `flash write failed` (the old log is kept). CLI: `timebase log`, `timebase log clear`, `timebase logdelta [ppm]`.

### 5.9 `setTimebase`: `logDeltaPpm` and the optional `ppm`

`setTimebase` can set the calibration (`ppm`, optional `source` `manual`|`ntp`, `dateUnix`, `durationS`; RAM only until `saveTimebase`), the log threshold (`logDeltaPpm`), or both. **At least one of `ppm` and `logDeltaPpm` is required** (`badRequest` `missing ppm` otherwise); **`ppm` is optional when `logDeltaPpm` is present**, in which case the stored calibration is left untouched.

- `source` is matched case-insensitively; besides `manual` and `ntp` the firmware also accepts `none` (stored as given), although its error text says "manual or ntp"; `pps` is refused. A `ppm` of exactly 0 with source `manual` is stored with `calSource` `"none"`. `dateUnix` 0 (or absent) stores the GPS UTC when known, else 0.
- Everything is validated before anything is applied: `logDeltaPpm` must be 0.05..10 (`badParam` `logDeltaPpm must be 0.05..10`); `source` `manual` or `ntp` (`badParam`); `ppm` finite and within ±1000 (`badParam` `ppm must be -1000..1000`, also for a date or duration outside 0..4294967295).
- `logDeltaPpm` is **persisted to flash** (DEVICE region), so it is refused with `busy` while the USB audio stream is open (and can fail with `flashError`). A request with only `ppm` is RAM only and **never** replies `busy`, also while audio streams. The write is **skipped when the value equals the current one**, so a client that always sends it neither wears flash nor gets `busy`. If the flash write is refused, nothing else of the request is applied.
- The reply is `Timebase` with the new `logDeltaPpm`.

### 5.10 Link and ownership ops (H7)

**`claim`.** Takes control for the requesting link (§4.6). Request `{ "takeover"?: boolean }`, default `false`.

```json
{ "v": 1, "id": 12, "op": "claim", "takeover": true }
```

Result `{ "owner": "ble", "previous": "usbHid" }`. `owner` is the requesting link. `previous` is the owner before the request, or `null` when nobody owned control. Without `takeover`, a refused claim replies `linkBusy` (`control held by usbHid` / `control held by ble`) and changes nothing. A `takeover` claim always succeeds.

**`getLink`.** Link status and control ownership. The `ble` object exists only on `TG_BLE` builds.

```json
{
  "usbHid": { "mounted": true },
  "ble": {
    "enabled": true, "name": "TG-1A2B", "address": "…", "advertising": false, "connected": true,
    "mtu": 185, "connIntervalUs": 15000, "connParamStep": 1, "connParamRejects": 0, "rxFrames": 412, "txFrames": 410, "rxDropped": 0,
    "rxInvalid": 0, "txRetries": 3, "disconnects": 1, "lastDisconnectReason": 19,
    "core": 1, "core1StackFree": 5120
  },
  "control": { "owner": "ble", "self": true, "leaseMs": 10000 }
}
```

| Field | Meaning |
|---|---|
| `usbHid.mounted` | The USB device is enumerated and the HID interface is mounted |
| `ble.enabled` | BLE is on (the CLI `ble off` turns it off, `TG_SERIAL_CLI` builds) |
| `ble.name`, `ble.address` | Advertised name `TG-XXXX` and the Bluetooth address, as text |
| `ble.advertising`, `ble.connected` | Advertising now; a central is connected with notifications on |
| `ble.mtu`, `ble.connIntervalUs` | Negotiated ATT MTU and current connection interval (µs) |
| `ble.connParamStep`, `ble.connParamRejects` | The device asks the central for a 15 ms interval 1 s after connect. Step: `0` not asked yet, `1` asked for 15 ms, `2` asked for 15 to 30 ms after a rejection, `3` gave up after a second rejection. `connParamRejects` counts rejections. A central may accept and still choose another interval; `connIntervalUs` shows the result |
| `ble.rxFrames`, `ble.txFrames` | Frames received on RX and sent on TX |
| `ble.rxDropped` | RX frames dropped because the RX ring was full |
| `ble.rxInvalid` | RX writes rejected (longer than 63 bytes, or a bad length) |
| `ble.txRetries` | TX sends held back by backpressure |
| `ble.disconnects`, `ble.lastDisconnectReason` | Links lost since boot; the HCI reason code of the last loss (0 = none) |
| `ble.core` | Core that runs the radio: `0` or `1` (`TG_BLE_CORE`) |
| `ble.core1StackFree` | Free bytes of core 1's stack (high-water mark). Present only when `ble.core` is `1` |
| `control` | Same shape as `hello.control` |

`getLink` follows the ownership rules of §4.6, like the other owner-only ops: a non-owner gets `linkBusy` while another link holds a live lease (10 s without a request from the owner); otherwise the request auto-claims control for the requesting link. To read who owns control without claiming, use `hello.control` (§5.1).

**`getDiag`.** The diagnostic counters of the firmware, with camelCase names: the fields of `diag_counters_t` in [`include/diagnostics.h`](../include/diagnostics.h), converted from `snake_case` (`dma_overrun_count` → `dmaOverrunCount`, `pcm1808_dma_block_overruns` → `pcm1808DmaBlockOverruns`). Most fields only increase. The ADC and ring `min` / `max` / `mean` fields and the PCM peak fields are extrema of the current diagnostic window. `getDiag` follows the same ownership rule as `getLink` (§4.6). The full field list is `HidDiag` in [`hid/timegrapher_hid.d.ts`](hid/timegrapher_hid.d.ts). It replaces the CLI `diag` command for hardware tests.

---

## 6. `getState` / `patchState`

### 6.1 Full state shape

```json
{
  "bypass": true,
  "blocks": {
    "levelMeter": { "enabled": true, "params": {} },
    "gain": { "enabled": true, "params": { "db": 12 } },
    "limiter": { "enabled": true, "params": {} },
    "filterBank": { "enabled": true, "params": { "preset": 1 } },
    "eventDetect": {
      "enabled": true,
      "params": { "threshold": 6, "prerollMs": 25, "tailMs": 25 }
    },
    "beatRate": { "enabled": true, "params": { "liftAngle": 51, "vph": 0, "calPpm": 0, "windowS": 6, "avgS": 10, "timingLandmark": 0, "settleS": 0 } },
    "noiseFloor": { "enabled": true, "params": {} },
    "eventLog": { "enabled": false, "params": {} },
    "softGate": {
      "enabled": false,
      "params": { "floorDb": -12, "attackMs": 1, "releaseMs": 40 }
    },
    "notch": { "enabled": true, "params": { "hz": 60 } },
    "agc": {
      "enabled": false,
      "params": { "target": 0.35, "tauS": 1.5, "gainDb": 0 }
    },
    "firBank": { "enabled": false, "params": { "preset": 0 } },
    "spectrum": { "enabled": false, "params": { "stride": 1 } }
  },
  "quartz": { "mode": "motor", "gateS": 60, "forkHz": 360, "tsMethod": "threshold", "kSigma": 8 }
}
```

Notes:

- `blocks` always holds all 13 blocks of `caps.blocks`, in that order; `bypass` (the master gate) is the top-level key, not a block. A param that the device cannot read is omitted from `params`.
- `bypass` is the master process-path gate (analysis may still run when `true`).
- `agc.params.gainDb` is **read-only** telemetry of the live AGC gain; `patchState` ignores writes to it.
- Enabling a process block clears bypass (same rule as CLI / `control_api_set_block_enabled`).
- `quartz` holds the quartz-timing settings (§5.3). It is present only when `hello.caps.quartz` is `true`.

`quartz` keys:

| Key | Type | Values | Default |
|---|---|---|---|
| `mode` | string | `"mechanical"` \| `"motor"` \| `"lcd"` \| `"fork"` (case-insensitive on input) | `"mechanical"` |
| `gateS` | number, s | `0` = until two inhibition events, or 3..960 (motor only) | `0` |
| `forkHz` | number, Hz | `300`, `341.333` (= 1024/3; any value within 0.01 Hz selects it), `360`, `480`, `720` (fork only) | `360` |
| `tsMethod` | string | `"threshold"` (50 % leading-edge crossing) \| `"xcorr"` (template cross-correlation); case-insensitive, and `0` / `1` are accepted on input (motor only) | `"threshold"` |
| `kSigma` | number | Motor detection threshold in noise sigmas, 4..20 | `8` |

Settings are not persisted; they return to the defaults on reboot.
### 6.2 Merge-patch semantics

`patchState` applies a **JSON Merge Patch**-style object:

- Omitted keys → unchanged.
- Present `bypass` → set.
- Present `blocks.<name>.enabled` → enable/disable that block.
- Present `blocks.<name>.params.<key>` → set that param.
- Present `quartz.<key>` → set that quartz setting. Within `quartz` the numeric settings and `tsMethod` are applied first and `mode` last, so a mode change starts its measurement with the new settings. Unknown `mode` / `tsMethod` names are rejected before anything in `quartz` is applied. Writing the current value again is a no-op (it does not restart the measurement), so echoing `getState` back is safe.
- Order across the patch: `bypass`, then `quartz`, then `blocks`.
- Response `result` is always the **full post-patch** `getState` snapshot (authoritative UI refresh without a second round-trip).

Example request:

```json
{
  "v": 1,
  "id": 3,
  "op": "patchState",
  "patch": {
    "bypass": false,
    "blocks": {
      "gain": { "enabled": true, "params": { "db": 6 } },
      "notch": { "enabled": true, "params": { "hz": 60 } }
    }
  }
}
```

Quartz example (select motor timing with a 60 s gate):

```json
{ "v": 1, "id": 4, "op": "patchState", "patch": { "quartz": { "mode": "motor", "gateS": 60 } } }
```

A rejected quartz value replies `badParam` with the field in the message, e.g. `{"code":"badParam","message":"quartz.gateS must be 0 or 3..960"}`. On a build without quartz, anything but `mode: "mechanical"` replies `badParam` / `quartz timing needs the PCM1808 source`.

### 6.3 Name and param alias table

JSON uses camelCase. Firmware maps to existing C/CLI names:

| JSON block key | CLI / `control_api` name |
|---|---|
| `levelMeter` | `level_meter` |
| `filterBank` | `filter_bank` |
| `eventDetect` | `event_detect` |
| `beatRate` | `beat_rate` |
| `noiseFloor` | `noise_floor` |
| `eventLog` | `event_log` |
| `softGate` | `soft_gate` |
| `gain` | `gain` |
| `limiter` | `limiter` |
| `notch` | `notch` |
| `agc` | `agc` |

The authoritative block / parameter name table is `src/dsp_blocks.cpp`
(`dsp_block_info()`); the tables below mirror it.

| JSON param | C / CLI param |
|---|---|
| `db` | `db` |
| `preset` | `preset` |
| `threshold` | `threshold` |
| `prerollMs` | `preroll_ms` |
| `tailMs` | `tail_ms` |
| `hz` | `hz` |
| `floorDb` | `floor_db` |
| `attackMs` | `attack_ms` |
| `releaseMs` | `release_ms` |
| `target` | `target` |
| `tauS` | `tau_s` |
| `gainDb` | `gain_db` (get only) |
| `stride` | `stride` (`spectrum`) |
| `liftAngle` | `liftAngle` / `lift_angle` |
| `vph` | `vph` (`0` = auto nearest A/h) |
| `calPpm` | `calPpm` / `cal_ppm` (get only over HID) |
| `windowS` | `windowS` / `window_s` |
| `avgS` | `avgS` / `avg_s` |
| `timingLandmark` | `timingLandmark` / `timing_landmark` |
| `settleS` | `settleS` / `settle_s` |

Quartz settings (`patchState.quartz`, §6.1) map to `control_api_set_quartz_mode` / `control_api_set_quartz_param` (CLI `quartz …`):

| JSON key | C / CLI name |
|---|---|
| `quartz.mode` | `control_api_set_quartz_mode` (`mechanical` / `motor` / `lcd` / `fork`) |
| `quartz.gateS` | `gate` |
| `quartz.forkHz` | `forkHz` / `fork_hz` |
| `quartz.tsMethod` | `tsMethod` / `ts_method` (`threshold` = 0, `xcorr` = 1) |
| `quartz.kSigma` | `kSigma` / `k_sigma` |

`filterBank.preset` and `firBank.preset`: `0`=bypass/off, `1`=wide, `2`=medium, `3`=narrow (writing a preset also sets the block's `enabled` to `preset != 0`).  
`spectrum.stride`: integer 1..16 (analyse every n-th block).  
`beatRate.vph`: `0` (auto) or 3600..72000.  
`agc.target`: above 0 up to 1; `agc.tauS`: 0.05..30.  
`notch.hz`: `50` or `60` only.  
`beatRate.liftAngle`: degrees, 10..90 (default 51).
`beatRate.calPpm`: sample-clock correction in effect, in ppm (+ = clock fast; the PPS
value while PPS is locked). Read-only here: `patchState` ignores it, so echoing state
back cannot overwrite a stored NTP calibration. Change it with `setTimebase`.
`beatRate.windowS`: rate/beat-error fit window in seconds, 2..30 (default 6).  
`beatRate.avgS`: integration time of the reported long average (`avgLong`, and the OLED readout) in seconds, 1..60 (default 10). Changing it keeps the sample history, so a longer window fills in as ticks arrive. Not persisted.  
`beatRate.timingLandmark`: which landmark of each beat is timed for the rate fit: `0` = the unlock (default), `1` = the drop. The drop is timed by a beat template (the shape of the whole beat, learned per side from the first 16 beats and then updated slowly) and not by a single peak, so it does not jump between the unlock and the drop when their loudness changes; on the ten recorded test clips the fit jitter fell on nine (by 1.5 to 8 times) and rose on one clipped recording. With `1` the readings start as early as with `0`: until both sides' templates hold 16 beats (about 5 s at 21,600 A/h) the fit runs on unlock times (`timingSource` `"unlock"`), and its readings are published and averaged like any other. Then comes the handover: each stored point of the fit is shifted by its side's running mean of (drop - unlock), and later beats use drop times (`timingSource` `"drop"`, `handovers` +1). The handover is not a cut and does not restart the fit or the stretch, so the fit's rate is continuous.

The rate and beat-error samples of the rolling means (`avg1s`, `avgLong`) taken while the stretch was provisional (under 10 s, partly unlock-timed) are dropped when `settled` becomes `true`: on a noisy recording a 5 s rate is off by tens of s/d and would otherwise stay in a long average. The averages therefore restart at that moment (invalid for about the next 0.3 s, then refilled from settled readings). Amplitude samples are kept. After the handover beat error comes from drop times, so it includes any difference in lift between the two sides; it can read higher than the unlock-based value on a watch whose two sides lift differently. A beat the template cannot time (correlation below 0.4, or too old for the 82 ms envelope ring) keeps its unlock time plus that side's mean lift; `dropFallbacks` counts them. Changing the value restarts the fit and clears the averages. Not persisted.  
`beatRate.settleS`: seconds, 0..60 (default 0). After a cut (§8.2) the first `settleS` s of the new stretch do not enter the rolling means (`avg1s`, `avgLong`), for rate, beat error and amplitude alike. The instant values are not affected. Not persisted.  
`gain.db`: -40..40 (writing it also enables `gain` and clears bypass). `eventDetect.threshold`: 1..100 (× noise floor);
`prerollMs`: 0..100; `tailMs`: 1..200. `softGate.floorDb`: -60..0;
`attackMs`: 0.1..50; `releaseMs`: 1..500. Non-finite values are rejected.
An out-of-range value makes `patchState` reply `badParam`; fields applied
earlier in the same patch are not rolled back.

---

## 7. Subscribe / push model

Default after connect: **no pushes** (quiet).

### 7.1 `subscribe` request

```json
{
  "v": 1,
  "id": 9,
  "op": "subscribe",
  "metricsHz": 10,
  "beatHz": 2,
  "timingHz": 2,
  "quartzHz": 1,
  "events": true,
  "errors": true,
  "quartzPulses": true,
  "beats": true,
  "beatsPacked": true
}
```

| Field | Meaning | Caps |
|---|---|---|
| `metricsHz` | Periodic `notify` / `op: "metrics"` | `0` = off; max **20** |
| `beatHz` | Periodic `notify` / `op: "beat"` | `0` = off; max **5** |
| `timingHz` | Periodic `notify` / `op: "timing"` | `0` = off; max **5** |
| `events` | Push when new analysis events land | boolean; batch ≤ 8 per notify |
| `errors` | Push when new faults / auto-ejects land | boolean |
| `quartzHz` | Periodic `notify` / `op: "quartz"` | `0` = off; max **5** (`caps.quartzHzMax`); forced to 0 when `caps.quartz` is false |
| `quartzPulses` | Push each new motor pulse (`op: "quartzPulse"`) | boolean; forced to `false` when `caps.quartz` is false |
| `beats` | Push new per-beat records (`op: "beats"`, §5.6) | boolean; batch ≤ 8 per notify |
| `beatsPacked` | Push new per-beat records in the packed format (`op: "beatsPacked"`, §5.6.1); replaces `beats` when both are set | boolean; batch ≤ 32 per notify; needs `caps.beatsPacked` |

Omitted subscribe fields leave the previous value unchanged; a rate is read as an integer (fractions truncated), a negative rate is 0 and a rate above its cap is clamped to the cap (never an error). A field of the wrong type is ignored. Response `result` echoes the full active subscription object (`metricsHz`, `beatHz`, `timingHz`, `quartzHz`, `events`, `errors`, `quartzPulses`, `beats`, `beatsPacked`).

Every `subscribe` re-seeds the event, error, quartz-pulse and beats baselines, so a subscribe never replays history: only events, faults, pulses and beats that land after it are pushed. It also restarts the four periodic timers, so the first periodic push of every active stream comes one full period after the `subscribe` reply.

Only one message is in flight at a time. When pushes compete, the order each poll is metrics, beat, timing, quartz, quartzPulse, beats, event, error; a periodic push that finds the link busy fires on a later poll, and event / error / pulse pushes are tracked against their baselines, so they are delayed, not lost (unless more than the log capacity arrives meanwhile).

### 7.2 Notify payloads

**metrics** — same fields as `getMetrics` / `DspMetrics`.

**timing** — same fields as `getTiming` / `DspTiming` (`dspBlockUs*`, `loopBusyPct`, underrun/short/ring proxies).

**beat** — same fields as `getBeat` / `DspBeatRate`.

**quartz** — same fields as `getQuartz` / `DspQuartz` (§8.5), at `quartzHz`.

**quartzPulse** — one motor pulse (`DspQuartzPulse`, §8.6), pushed once its 50 ms pulse group has closed, so `extras` and `clipped` are final. Pulses arrive oldest first, one per notify; at most one per motor step (≥ 1 s). The device tracks what it has sent by (`measurementId`, `sequence`): after a restart every pulse of the new measurement is pushed from sequence 0.

```json
{
  "v": 1,
  "notify": true,
  "op": "quartzPulse",
  "data": { "measurementId": 7, "sequence": 41, "sampleIndex": "123456789012", "tS": 41.0000123, "peak": 512345,
            "snr": 51.7, "polarity": -1, "extras": 0, "clipped": false, "closed": true }
}
```

**beats** — new per-beat records, **oldest first**, at most 8 (`DspBeat`, §8.7). Baseline, ordering and stream-reset rules: §5.6.

```json
{ "v": 1, "notify": true, "op": "beats", "data": { "beats": [ /* DspBeat */ ] } }
```

**beatsPacked** — the same beats in the packed binary format, **oldest first**, at most 32 per notify (§5.6.1). Sent instead of `beats` when `subscribe.beatsPacked` is set.

```json
{ "v": 1, "notify": true, "op": "beatsPacked", "data": { "p": "AQgrCAAA…" } }
```

**event** — one or more new events, **oldest first** (like `beats` and `quartzPulse`; earlier firmware sent newest first, at most 8). A notify carries as many unsent events as fit one message (at least 5, usually 8 or more); the rest follow in the next polls, so none is lost while the event ring (32) has not lapped. The cursor is the `peakSample` of the last event sent (events carry no seq; their windows close in order so `peakSample` increases along the log): events with a larger `peakSample` are new. If more than 32 events land between two pushes, the evicted ones are gone and the push resumes at the oldest event still held; if the newest held event is older than the cursor (sample clock restart) the cursor resets and every held event is sent:

```json
{
  "v": 1,
  "notify": true,
  "op": "event",
  "data": { "events": [ /* DspEvent */ ] }
}
```

**error** — structured fault (also covers auto-eject; no CLI text dependency). Only the **latest** fault is pushed, so if two faults land between two polls the older one is not notified (read `getErrors`):

```json
{
  "v": 1,
  "notify": true,
  "op": "error",
  "data": {
    "block": "filterBank",
    "reason": "underrun",
    "uptimeMs": 12345,
    "sampleIndex": "0"
  }
}
```

`reason` strings: `none` | `underrun` | `usbStall` | `overload`.

**controlReleased** (H7) — sent to the **previous owner** when another link takes control (`claim` with `takeover`, or an auto-claim after the lease expired), if its link is still connected (§4.6). `to` is the new owner. The previous owner's subscriptions have stopped when this arrives; show that control moved and do not resend subscriptions unless the user takes control again:

```json
{ "v": 1, "notify": true, "op": "controlReleased", "data": { "to": "ble" } }
```

---

## 8. Data types (JS conventions)

| Kind | Convention |
|---|---|
| Levels | float full-scale ≈ `[-1, 1]` |
| Gains | dB as JSON numbers |
| Booleans | JSON `true`/`false` (never `0`/`1`) |
| Sample indices | **decimal strings** (e.g. `"startSample": "123456789012"`) so JS does not lose `uint64` precision above 2^53 |
| Enums in JSON | strings for reasons; numbers OK for `preset` / `hz` where already numeric in CLI |

### 8.1 `DspMetrics`

```json
{
  "prePeak": 0.4,
  "postPeak": 0.35,
  "preRms": 0.02,
  "postRms": 0.02,
  "noiseFloor": 0.0001,
  "agcGainDb": 0,
  "clipCount": 0,
  "limiterCount": 0
}
```

### 8.2 `DspBeatRate`

Unlock-based Witschi-style timing (rate s/d, beat error ms, amplitude °).  
`beatRate.params` `liftAngle`, `vph` (`0` = auto), `windowS`, `avgS`, `timingLandmark` (`0` unlock, `1` drop) and `settleS` are set via `patchState`; `calPpm` is reported there but set with `setTimebase`.

Rate and beat error come from a least-squares fit of sub-sample unlock times over the last `windowS` seconds, not from single intervals. Each tick refines the fit, so the instant values are already smoothed over that window. Fields that describe the fit and the timebase:

| Key | Meaning |
|---|---|
| `state` | `"listening"`: no tick since the stream reset, or the capture rate is unverified. `"locking"`: ticks seen, but there is no lock on a nominal rate or the current stretch spans less than 5 s. `"locked"`: locked and the stretch spans at least 5 s (a rate can be reported; see `settled`). `"noWatch"`: the no-watch rule fired (below) and no lock has held a stretch of 2 s since. `locked` may turn `true` again while `state` is still `"noWatch"`; that lock carries no rate |
| `analysis` | `"on"` or `"off"`. `"off"` in `usbMic` mode (§5.4): the timing engine does not run, so `rateValid`, `beatErrorValid`, `amplitudeValid` and all `*Valid` flags are `false`, `stretchS` is `0`, no beats are recorded, and the rate, beat-error and amplitude values are `0`. `locked`, `vphNearest`, `vphRaw`, `confidence`, `intervalSamples` and `state` stay live: `state` is then `"listening"`, `"locking"` or `"locked"` and follows the lock alone |
| `settled` | `true` once the current stretch spans at least 10 s: rate and beat error are then final. Before that (5 to 10 s of stretch) they are provisional and a host may grey them out |
| `cuts` | Cuts since the stream reset (below); the reset itself is not counted |
| `lastCut` | Reason of the latest cut: `"none"`, `"reset"` (after a stream reset), `"gap"`, `"jump"`, `"rate"`, `"clock"`, `"lockLost"` or `"noWatch"` |
| `stretchS` | Span of the current stretch in seconds (first to latest accepted beat since the last cut, on the corrected clock); `0` when none is open |
| `sourceHz` | Analysis sample rate used for all timing math |
| `sourceHzMeasured` | Measured capture frame rate (PCM1808), `0` where the rate is fixed by design |
| `timebaseOk` | `false` while the capture rate is unverified or unsupported. In that case `locked` stays `false` and no timing is reported |
| `ratePoints` | Ticks used in the fit (0 = no fit yet; at least 8 needed) |
| `jitterUs` | RMS scatter of the timed landmark (unlock, or drop with `timingLandmark` = 1) about the fit (µs). This is a signal-quality indicator |
| `timingSource` | `"unlock"` while the fit runs on unlock times, `"drop"` once it runs on drop times. With `timingLandmark` = 1 it starts as `"unlock"` (so rate and beat error are published as early as with the unlock landmark) and changes to `"drop"` at the handover (below). Always `"unlock"` with `timingLandmark` = 0 |
| `handovers` | Handovers from unlock to drop times since the stream reset |
| `dropTimedBeats` | Beats timed at the drop by the beat template since the last stream reset; `0` with `timingLandmark` = 0 |
| `dropFallbacks` | Beats that should have been timed at the drop but were not (template still learning, correlation below 0.4, or the beat outside the envelope ring) since the last stream reset |
| `beatErrorSignedMs` | Signed beat error (ms): positive when the Tick comes late. Tick is the side that came late over the first seconds of the watch (decided when the stretch first reads, within 0.1 ms of zero the side of the stretch's first beat) and keeps that label until the nominal rate changes, so the value starts positive and can turn negative. Valid with `beatErrorValid`; `beatErrorMs` is its magnitude. With `timingLandmark` = 0 it equals `beatErrorMs` |
| `beatErrorSource` | `"unlock"` or `"drop"`. `"unlock"`: the unlock landmark (the unlock-timed fit, or, with the drop landmark, the median over 2 s shape windows of the drop beat error corrected by each side's unlock-to-drop time). `"drop"`: the drop-timed fit, used while fewer than 3 windows exist or the windows disagree in sign beyond their spread |
| `amplitudeTickDeg` / `amplitudeTockDeg` | Amplitude of the Tick and the Tock side (degrees): median over the 2 s shape windows of the averaging span. `null` unless `timingLandmark` = 1 and the stretch has a window with that side valid. The Tick side is the one `beatErrorSignedMs` calls late |
| `amplitudeWindows` | `{ "used", "total" }`: shape windows of the averaging span that gave an amplitude on both sides, and all windows of the span that held a beat. The window median is the `avgLong.amplitudeDeg` reading |
| `calPpm` / `calSource` | Sample-clock correction applied to all timing, and where it came from (`none` / `manual` / `ntp` / `pps`) |

Top-level `rateSDay` / `beatErrorMs` / `amplitudeDeg` are the **instant** view (latest fit). Nested windows:

| Key | Meaning |
|---|---|
| `instant` | Same values as the top-level rate/BE/amp fields |
| `avg1s` | Arithmetic mean of instant samples over the last ~1 s |
| `avgS` | Integration time of `avgLong` in seconds (the `beatRate.avgS` param) |
| `avgLong` | Arithmetic mean of instant samples over the last `avgS` seconds. Replaces the fixed-10 s `avg10s` of earlier firmware |

**Shape windows (`timingLandmark` = 1, opentimegrapher M4 / M5 / M6).** The beats stage B timed at the drop are averaged per side (Tick / Tock) in 2 s windows laid end to end from the start of the stretch (a side needs at least 3 beats; plain mean of the aligned envelopes, where OTG uses a per-sample trimmed mean). On each averaged shape the drop is the half-height crossing of its rising edge, the unlock the half-height crossing of the first steady rise, and their distance the lift time of that side. `avgLong.amplitudeDeg` is then the median over the windows of the averaging span (`avgS`) of the mean of the two sides' amplitudes, and `amplitudeTickDeg` / `amplitudeTockDeg` the per-side medians. The window beat error is `BE_drop - lift_tick + lift_tock`; `beatErrorMs` and `avgLong.beatErrorMs` are the median over the windows. `instant.amplitudeDeg` and the per-beat `amplitudeDeg` of `getBeats` stay single-beat values. Before the first window closes, and with `timingLandmark` = 0, the previous per-tick figures apply. Nothing spans a cut, and `settleS` applies to windows too.

Beat-error angle (opentimegrapher M5): `beatErrorAngleDeg` = `amplitudeDeg` × π × `vphNearest` / 7200 × `beatErrorMs` / 1000, in degrees. It puts beat errors of different amplitudes on one scale. Valid (`beatErrorAngleValid`) when `beatErrorValid`, `amplitudeValid` and a lock all hold. Each window carries it too: a rolling value is the mean of the per-tick angles in the window, with at least 2 contributing ticks.

**Cuts.** A stretch is a run of accepted beats with no cut in it. A cut ends the stretch: the next one starts with an empty fit, empty rolling means and an empty amplitude history, and nothing is fitted or averaged across the cut. When the nominal rate changes the beat templates (`timingLandmark` = 1) are forgotten too. A cut is counted (`cuts`, `lastCut`) only if it ended a stretch that had beats. Reasons:

| `lastCut` | Cut when |
|---|---|
| `gap` | More than 2 s passed without an accepted beat |
| `jump` | Four consecutive beats were off the current grid by more than 0.3 of a half-period (the fit restarted) |
| `rate` | The nominal A/h changed (another watch or another rate) |
| `clock` | The timebase correction changed by more than 0.5 ppm since the last beat, or the capture rate became unverified or verified |
| `lockLost` | The lock on a nominal rate was lost |
| `noWatch` | The no-watch rule fired |

**Short stretch.** `rateValid` and `beatErrorValid` (instant and rolling) stay `false` until the current stretch spans at least 5 s (`stretchS` ≥ 5). From then on they are provisional (a rate from a few seconds of beats can be several s/d out) until the stretch spans 10 s, when `settled` becomes `true`. Amplitude is not held back.

**No watch.** Over the last 10 s of the stretch (once it spans 5 s), the rule fires if fewer than half the expected beats were accepted, or if the RMS scatter of the beat times about the fit is at least 2 % of the nominal half-period (3.3 ms at 21,600 A/h). Rejected detections count in the scatter at their distance from the fit, and the worst 1 in 8 events are left out, so one knock, a short dropout or the stray detections of a real recording do not fire it. When it fires, `locked` becomes `false`, `vphNearest` `0`, the stretch is cut (`lastCut` = `"noWatch"`) and `state` reads `"noWatch"`. Lock can return through the normal path (four fresh intervals within 4 % of a standard rate); `state` leaves `"noWatch"` once a lock has held a stretch of 2 s.

Rolling windows need ≥2 contributing samples per metric before `*Valid` is true. `samples` is the contributing tick count (max of the three metric counts).

```json
{
  "vphRaw": 18012.5,
  "vphNearest": 18000,
  "confidence": 0.95,
  "locked": true,
  "state": "locked",
  "analysis": "on",
  "cuts": 0,
  "lastCut": "reset",
  "stretchS": 21.4,
  "intervalSamples": 4800,
  "eventsSeen": 12,
  "intervalsHeld": 8,
  "sourceHz": 24000,
  "sourceHzMeasured": 24000.012,
  "timebaseOk": true,
  "ratePoints": 47,
  "jitterUs": 18.5,
  "dropTimedBeats": 0,
  "dropFallbacks": 0,
  "beatErrorSignedMs": 0.35,
  "beatErrorSource": "unlock",
  "amplitudeTickDeg": null,
  "amplitudeTockDeg": null,
  "amplitudeWindows": { "used": 0, "total": 0 },
  "calPpm": -23.412,
  "calSource": "ntp",
  "rateSDay": 4.2,
  "beatErrorMs": 0.35,
  "amplitudeDeg": 288.0,
  "liftAngleDeg": 51,
  "liftTimeMs": 3.8,
  "beatErrorAngleDeg": 0.79,
  "rateValid": true,
  "beatErrorValid": true,
  "amplitudeValid": true,
  "beatErrorAngleValid": true,
  "instant": {
    "rateSDay": 4.2,
    "beatErrorMs": 0.35,
    "amplitudeDeg": 288.0,
    "beatErrorAngleDeg": 0.79,
    "samples": 1,
    "rateValid": true,
    "beatErrorValid": true,
    "amplitudeValid": true,
    "beatErrorAngleValid": true
  },
  "avg1s": {
    "rateSDay": 3.9,
    "beatErrorMs": 0.32,
    "amplitudeDeg": 286.5,
    "beatErrorAngleDeg": 0.72,
    "samples": 8,
    "rateValid": true,
    "beatErrorValid": true,
    "amplitudeValid": true,
    "beatErrorAngleValid": true
  },
  "avgS": 10,
  "avgLong": {
    "rateSDay": 3.7,
    "beatErrorMs": 0.30,
    "amplitudeDeg": 285.0,
    "beatErrorAngleDeg": 0.67,
    "samples": 72,
    "rateValid": true,
    "beatErrorValid": true,
    "amplitudeValid": true,
    "beatErrorAngleValid": true
  }
}
```

### 8.3 `DspEvent`

One analysis event (`getEvents` element, `event` notify element), newest first in `getEvents`, oldest first in the `event` notify. The four `*Sample` values are decimal strings; the encoded object is 216 bytes with small values and up to 345 bytes at the worst case (20-digit indices, 12-character floats). A `getEvents` reply (`{"events":[...]}` plus the envelope) stays under 2048 bytes by dropping the oldest requested events that do not fit (at most 5 worst-case events fit, 9 of the typical 214-byte ones).

```json
{
  "startSample": "1000",
  "peakSample": "1100",
  "unlockSample": "1020",
  "dropSample": "1100",
  "durationSamples": 200,
  "riseSamples": 100,
  "liftSamples": 80,
  "peakAbs": 0.5,
  "noiseFloor": 0.0001,
  "snrDb": 74.0,
  "energy": 1.2,
  "confidence": 0.8
}
```

### 8.4 `DspError`

```json
{
  "block": "gain",
  "reason": "underrun",
  "uptimeMs": 5000,
  "sampleIndex": "0"
}
```

### 8.5 `DspQuartz`

Quartz-watch timing snapshot (`getQuartz`, `quartzRestart`, `quartz` notify). Conventions for every rate below:

- **Rates are seconds per day (s/d); positive = the watch gains**, negative = it loses (handoff §8).
- **Calibrated vs uncal.** A field without `Uncal` has the sample-clock correction (`calPpm`, the same correction `getTimebase` reports) removed. A field named `*UncalSDay` is measured against the board clock with no correction. The relation is `cal = (uncal + 86400) · (1 + calPpm·1e-6) − 86400 ≈ uncal + 0.0864 · calPpm`. When no calibration exists (`calibrated: false`, `calSource: "none"`) the correction is 0 ppm and both are equal, and the UI must flag every rate as **uncalibrated**: an uncorrected board crystal can be off by ±20 ppm = ±1.7 s/d.
- **"Raw" / "without inhibition"** (handoff §5) means the oscillator rate measured within one inhibition segment, i.e. before the watch IC's digital trimming. It is **not** "uncalibrated". "With inhibition" is the rate averaged over whole inhibition cycles, i.e. what the wearer sees.
- Times (`elapsedS`, `spanS`, pulse `tS`) are seconds of signal (sample count / `sourceHz`), not wall clock.
- Non-finite values are reported as `0`; use the `*Valid` / `valid` flags.
- Exactly one of `motor` / `lcd` / `fork` is an object: the one for the active `mode` (it is present from the moment the mode is selected, with `valid: false` until it has a result). The other two are `null`; all three are `null` in mechanical mode.

Top level:

| Key | Type / unit | Meaning |
|---|---|---|
| `mode` | string | Active mode (§5.3) |
| `state` | string | `idle` \| `waitTimebase` \| `unsupportedRate` \| `acquiring` \| `measuring` \| `done` (§5.3) |
| `analysis` | string | `"on"` or `"off"`. `"off"` in `usbMic` mode (§5.4): nothing is measured, `state` is `idle`, `reason` is `"USB Audio mode"`, `rateValid` is `false` and `motor` / `lcd` / `fork` are empty (`valid: false`, zeros). `mode` still shows the selected setting |
| `measurementId` | uint32 | Identifies the current measurement. Increases by one on every (re)start (mode change, `quartzRestart`, a setting change that restarts the active mode, stream reset, source-rate or timebase change, sample loss, index gap); never repeats while the firmware runs (wraps after 2^32). A change means a new measurement: drop plots and pulse lists tied to the old one. Re-reading the same measurement, including a frozen `done` one, never changes it |
| `reason` | string | Why the state is what it is (e.g. `"waiting for 3 pulses"`, `"two inhibition events seen"`); `""` when nothing to add. Display only |
| `sourceHz` | Hz | Analysis sample rate (must be 48000 to measure) |
| `timebaseOk` | bool | Capture rate verified |
| `elapsedS` | s | Signal time since the measurement started; frozen in `done` |
| `gateS` / `forkHz` / `tsMethod` / `kSigma` | | Settings in effect (as in `getState.quartz`) |
| `sampleLossRestarts` | count | Capture sample-loss events (DMA overruns) that forced a restart since the mode was selected |
| `rateSDay` | s/d, + = gaining | Headline rate, calibrated: motor = `motor.rateWithSDay`, LCD / fork = their phase-slope rate |
| `rateValid` | bool | `rateSDay` is meaningful |
| `calPpm` | ppm, + = board clock fast | Correction applied to every calibrated rate. In `done`, the value the result was frozen with |
| `calSource` | string | `none` \| `manual` \| `ntp` \| `pps` |
| `calibrated` | bool | `false` when `calSource` is `none`: every rate is uncalibrated |
| `motor` | `DspQuartzMotor` \| null | Motor detail |
| `lcd` | `DspQuartzLcd` \| null | LCD detail |
| `fork` | `DspQuartzFork` \| null | Tuning-fork detail |

`motor` (stepper motor, handoff §4/§5; rate fields are 0 while `valid` is false):

| Key | Type / unit | Meaning |
|---|---|---|
| `valid` | bool | A fit exists (three or more pulses) |
| `pulses` | count | Pulses detected since the measurement started |
| `fitPulses` | count | Pulses in the fit (≤ 2048, sliding window) |
| `stepS` | s | Step interval snapped to the allowed list (1, 2, 2.5, 5, 10, 20, 30, 60) |
| `spanS` | s | First to last fitted pulse |
| `rateWithSDay` | s/d | Rate **with** inhibition, calibrated: the motor headline. Equals `rateWithoutSDay` unless `inhibition` is `corrected` |
| `rateWithUncalSDay` | s/d | Same, board clock |
| `rateWithoutSDay` | s/d | **Raw** rate (without inhibition: the oscillator), calibrated |
| `rateWithoutUncalSDay` | s/d | Same, board clock |
| `inhibition` | string | Handoff §5 status, see below |
| `inhibitionPeriodValid` | bool | Two or more inhibition events seen |
| `inhibitionPeriodS` | s | Median event spacing, 0 until valid |
| `events` | count | Inhibition jumps (> 20 µs) seen |
| `cyclesUsed` | count | Whole inhibition cycles in the corrected rate |
| `jumpLastUs` | µs | Size of the most recent jump, 0 if none |
| `jumpLastN` | count | `jumpLastUs / 122.07 µs`, rounded: the pulses deleted at 8192 Hz ("n"; watchmakers use it) |
| `jumpMeanUs` | µs | Mean jump size, 0 if none |
| `jumpMeanN` | count | Mean jump in 8192 Hz pulses, rounded |
| `residualRmsUs` | µs | RMS fit residual: per-pulse timing jitter |
| `polarityOffsetUs` | µs | Constant edge offset between the two coil halves (alternate pulses) |
| `clipped` | bool | A LEFT sample hit the ADC rail since the start |
| `clippedPulses` | count | Pulses whose group hit the rail |
| `extraPulses` | count | Detection / correction bursts logged (fault indication, handoff §3.1) |
| `noiseSigma` | ADC codes | Noise sigma after the 400 Hz high-pass; 0 until the detector has calibrated |
| `threshold` | ADC codes | Detection threshold, `kSigma × noiseSigma` |
| `peakSnr` | ratio | Median `peak / noiseSigma` over the recent pulses (≤ 16); 0 with no pulse |
| `hintWeakSignal` | bool | `peakSnr` below the weak-signal limit (≈ 30 µV pickup EMF): tell the user to reposition the watch (handoff §8) |
| `hintClipped` | bool | `clipped`: tell the user to open JP1 (handoff §8) |
| `noJumpSpanS` | s | Fitted span a measurement without a jump must reach before `inhibition` becomes `no_jump_long_gate` (twice the longest expected inhibition period). Engine constant, filled in every motor state; use it for a "no inhibition yet: x / 120 s" progress hint |
| `jumpThresholdUs` | µs | Interval excess that counts as an inhibition jump (20 µs, handoff §4). Engine constant |
| `rebuilding` | bool | The fit is replaying its pulse store (after a step change or a re-timed pulse); the rate fields are the last good result, not yet updated with the newest pulses. Transient (well under a second); a motor gate never freezes during it. Show a "refitting" note, keep the displayed rates |

`motor.inhibition` (handoff §5), and what a UI should show:

| Value | Situation | Show |
|---|---|---|
| `none` | Fewer than three pulses | No rate yet |
| `no_jump_long_gate` | No jump over a span ≥ 2 × the longest expected period (`noJumpSpanS`, 120 s) | Rate (no inhibition detected) |
| `may_include_inhibition` | No jump in a short gate | Raw rate, flagged "may include inhibition". Offer a longer gate |
| `need_one_more_cycle` | One jump | Raw rate plus a "need one more cycle" status |
| `correction_pending` | Two jumps, waiting for the next same-polarity pulse (transient) | Raw rate, "correction pending" |
| `corrected` | ≥ 2 jumps | Corrected rate as the headline; the raw rate, the period and the jump size in µs as details |

`lcd` (handoff §6):

| Key | Type / unit | Meaning |
|---|---|---|
| `valid` | bool | A phase-slope rate exists |
| `status` | string | `detecting_period` (collecting the 2 s record) \| `no_pattern` (no candidate passed; retries) \| `unsupported_rate` (pattern period does not fit the buffer) \| `tracking` (f0 known; phase blocks running) |
| `f0Hz` | Hz | Pattern rate 16 / 32 / 64 / 128, 0 until known |
| `harmonics` | number[] | Harmonic numbers k tracked (≤ 4) |
| `blocks` | count | 0.5 s phase blocks in the fit |
| `rateSDay` | s/d | Rate, calibrated (the oscillator: the LCD cannot see inhibition, so it may legitimately differ from a motor rate) |
| `rateUncalSDay` | s/d | Same, board clock |
| `retries` | count | Pattern searches restarted after `no_pattern` |
| `clipped` | bool | A RIGHT sample hit the rail since the start |

`fork` (handoff §7):

| Key | Type / unit | Meaning |
|---|---|---|
| `valid` | bool | A phase-slope rate exists (≥ 2 blocks) |
| `fNomHz` | Hz | Nominal fork frequency |
| `blocks` | count | 0.25 s phase blocks in the fit |
| `rateSDay` | s/d | Rate, calibrated |
| `rateUncalSDay` | s/d | Same, board clock |
| `clipped` | bool | A LEFT sample hit the rail since the start |

Example (motor, `ind_inhibition_60s`-like, gate "until 2 events"):

```json
{
  "mode": "motor", "state": "done", "analysis": "on", "measurementId": 7, "reason": "two inhibition events seen",
  "sourceHz": 48000, "timebaseOk": true, "elapsedS": 121.5,
  "gateS": 0, "forkHz": 360, "tsMethod": "threshold", "kSigma": 8, "sampleLossRestarts": 0,
  "rateSDay": 0.2059, "rateValid": true, "calPpm": -1.25, "calSource": "pps", "calibrated": true,
  "motor": {
    "valid": true, "pulses": 121, "fitPulses": 121, "stepS": 1, "spanS": 120,
    "rateWithSDay": 0.2059, "rateWithUncalSDay": 0.3139,
    "rateWithoutSDay": 0.9022, "rateWithoutUncalSDay": 1.0102,
    "inhibition": "corrected", "inhibitionPeriodValid": true, "inhibitionPeriodS": 60,
    "events": 2, "cyclesUsed": 1, "jumpLastUs": 488.3, "jumpLastN": 4, "jumpMeanUs": 488.3, "jumpMeanN": 4,
    "residualRmsUs": 1.65, "polarityOffsetUs": 0.12, "clipped": false, "clippedPulses": 0, "extraPulses": 0,
    "noiseSigma": 9900, "threshold": 79200, "peakSnr": 15.8, "hintWeakSignal": true, "hintClipped": false,
    "noJumpSpanS": 120, "jumpThresholdUs": 20, "rebuilding": false
  },
  "lcd": null,
  "fork": null
}
```

Size: the motor snapshot is ≈ 0.9 KB; the worst case (every counter at `UINT32_MAX`, 12-character floats, a 95-character reason) is 1346 bytes including the response envelope, within the 2048-byte limit (checked by `tests/test_hid_json.cpp`).

### 8.6 `DspQuartzPulse`

One recent motor pulse (`getQuartzPulses`, `quartzPulse` notify).

| Key | Type / unit | Meaning |
|---|---|---|
| `measurementId` | uint32 | `DspQuartz.measurementId` of the measurement the pulse belongs to |
| `sequence` | count | 0-based pulse number within that measurement (restarts at 0). Key merged pulse lists by (`measurementId`, `sequence`) |
| `sampleIndex` | decimal string | Absolute source index of the first sample over threshold (uint64) |
| `tS` | s | Leading-edge timestamp since the measurement started (sub-sample, 7 decimals = 0.1 µs). With `tsMethod: "xcorr"` it carries a constant template offset that cancels in rates |
| `peak` | ADC codes | \|y\| at the leading-spike peak after the 400 Hz high-pass |
| `snr` | ratio | `peak / noiseSigma` at detection |
| `polarity` | `1` \| `-1` | Sign of the leading spike; alternates every step |
| `extras` | count | Detection / correction bursts in this pulse's group |
| `clipped` | bool | A raw sample reached the ADC rail in this group |
| `closed` | bool | Its 50 ms quiet gap has passed (always `true` in `quartzPulse` notifies) |

`getQuartzPulses` returns at most 8 pulses (`caps.quartzPulsesMax`) although the device logs 16: a pulse is up to 214 bytes, so 16 could exceed the 2048-byte message limit. The worst-case 8-pulse reply is 1785 bytes with its envelope. Use the `quartzPulse` notify to see every pulse.

### 8.7 `DspBeat`

One tick as seen by the beat-timing fit (`getBeats`, `beats` notify): the raw material of a **tick-tock (paper-strip) chart**. There is one record per tick handed to the fit once the A/h lock is established.

```json
{ "seq": 1204, "gridId": 3, "sampleIndex": "123456789", "tS": 2571.8914583, "k": 842, "parity": 0,
  "uUs": -41.3, "nominalUs": 100000, "amplitudeDeg": 287.5, "amplitudeValid": true,
  "liftMs": 3.7, "status": "accepted" }
```

| Key | Type / unit | Meaning |
|---|---|---|
| `seq` | uint32 | Per-stream counter, **0 at a stream reset**; the next beat has `seq + 1` (§5.6) |
| `gridId` | uint32 | Bumps on every `started` (the first grid is 1). A new `gridId` means a new grid origin: start a new trace |
| `sampleIndex` | decimal string | Integer part of the sample time of the timed landmark (the unlock, or the drop with `beatRate.timingLandmark` = 1) |
| `tS` | s | Time of that landmark in seconds of signal (`sampleIndex` plus the sub-sample part, over the source rate), 7 decimals |
| `k` | int32 | Half-beat grid index since the grid origin (the `started` tick is `k = 0`). The fit window sliding never renumbers it |
| `parity` | `0` \| `1` | `k & 1`: even `k` = "tick", odd `k` = "tock" (arbitrary but stable within a grid) |
| `uUs` | µs | Residual **arrival time minus nominal-grid time**, `u = T − k · nominal` (see below) |
| `nominalUs` | µs | Nominal half-period of the grid (e.g. 100 000 for 18 000 A/h) |
| `amplitudeDeg` | ° | Per-tick amplitude; `0` when not valid |
| `amplitudeValid` | bool | The lift time gave 50..400° |
| `liftMs` | ms | Unlock-to-drop lift time; `0` if unavailable |
| `status` | string | `accepted` (joined the grid / fit), `started` (first point, or the grid restarted here), `rejected` (off-grid or non-increasing; **not** in the fit) |

**What `uUs` is.** `T` is the tick's landmark time (as `tS`) since the grid origin **with the sample-clock (timebase) correction applied**: scaled by `1 / (1 + ppm·1e-6)`, the same `Timebase.ppm` correction that the rate uses. So `uUs` is already calibrated. It is measured against the **nominal** grid, not against the fitted line; the origin tick has `uUs = 0`.

**Sign.** A watch that **gains** (positive s/d) has its ticks arrive **early**, so `u` **decreases** with time. The rate is the negative slope of `u`:

```text
rate_s_day = -(du/dt) * 86400          (u and t in seconds; u = uUs * 1e-6, t = tS)
```

Equivalently the slope per beat is `-rate_s_day / 86400 * nominal`. A watch that loses has `u` increasing.

**Tick-tock chart.**
1. Use `tS` (or the beat index `k`) as the x axis and the **lead `-uUs`** (early = up) as the y axis, as a conventional timegrapher strip does. The trace then slopes with the sign of the rate: a gaining watch rises, a losing one falls, matching the `rateSDay` trend (`rate_s_day = +d(-u)/dt * 86400`). Plotting `uUs` itself gives the same picture upside down. Start a new trace whenever `gridId` changes (and drop everything on a `seq` reset, §5.6).
2. Colour or split by `parity`: draw tick (`0`) and tock (`1`) as two series.
3. In a healthy watch both series are two parallel sloping lines. The common slope is the rate (formula above); the **vertical separation of the two lines is the beat error** (about 2 times the offset of each from their mean).
4. Plot `rejected` beats as marks (or hide them): their `k` is the nearest grid slot to the last accepted tick (it can equal or be lower than an earlier `k`) and `uUs` is the true distance to that slot, up to about half a beat, so they show the off-grid misses. They are not part of the rate.
5. If the timebase ppm changes while a grid is running, `uUs` steps: treat it as a new grid (clear the trace) rather than as watch behaviour.

`getBeat` (`DspBeatRate`, §8.2) remains the authority for the windowed rate, beat error and amplitude; `DspBeat` is for plots and diagnostics.

---

## 9. Firmware architecture

| Module | Role |
|---|---|
| [`hid_control.cpp`](../src/hid_control.cpp) / [`hid_control.h`](../include/hid_control.h) | USB HID adapter: report descriptor, the `UsbHidLink`, the USB `Session` (framing, dispatch and encoders are in `control::`, below) |
| [`hid_json.cpp`](../src/hid_json.cpp) / [`hid_json.h`](../include/hid_json.h) | Host-testable encoders: bounded `appendf`, string sanitising, `hello` (caps and pipeline), `DeviceStatus`, beats, `GpsInfo`, `TimebaseLog` pages, quartz payloads (`tests/test_hid_json.cpp`) |
| [`device_mode.cpp`](../src/device_mode.cpp), [`flash_store.cpp`](../src/flash_store.cpp), [`timebase.cpp`](../src/timebase.cpp) | Device mode / run latch, the single owner of the flash sector, timebase log policy and deferred write (see [AS_BUILT_ARCHITECTURE.md](AS_BUILT_ARCHITECTURE.md)) |
| `Adafruit_USBD_HID` in `hid_control_usb_begin()` | Vendor report descriptor (Usage Page `0xFFA0`), Input+Output EP |
| [`usb_audio_init`](../src/usb_audio.cpp) | Calls `hid_control_usb_begin()` before re-enumeration |
| [`TimegrapherStand_main`](../src/TimegrapherStand_main.cpp) | `hid_control_task()` each loop |

JSON on device: fixed-schema decoder for known keys; encoder via `snprintf`. One inbound reassembly buffer per link; TX fragments pumped when `hid.ready()`.

H7 module split ([plan §5.1](plans/BLE_TRANSPORT_PLAN.md)). `hid_control.cpp` is only the USB HID adapter; the protocol logic lives in `control::`:

| Module | Role |
|---|---|
| `control/frame` (`include/control/frame.h`) | Framer: reassembler and fragmenter, pure and host-tested (§3) |
| `control/link`, `control/handler`, `control/session` | Abstract `Link` (one transport endpoint), `Handler` / `Reply` (protocol brain and its reply sink), and one `Session` per link (reassembly, fragmentation, one request at a time, link-down detection) |
| `control/arbiter` | `Arbiter`: ownership state machine of §4.6 (none / usbHid / ble, lease, takeover). `ArbiterPort`: the per-link handler a session talks to. It answers `claim` and `linkBusy`, queues `controlReleased`, and forwards everything else to the control handler |
| `control/control_handler`, `control/arbiter_port` | The one op dispatch, encoders and subscription engine (moved from `hid_control.cpp`), and the firmware singletons: the arbiter, the control handler and one port per link |
| `control/json_scan` | Request scanners (`find_key`, `get_*`, `extract_object`, `parse_i64`), shared by the handler, the port and the tests |
| `ble_control` (`include/ble_control.h`, `src/ble_control.cpp`) | BLE link: GATT service, rings, advertising, `getLink` status. Built only with `TG_BLE=1` ([`BLE_CONTROL_PLANE.md`](BLE_CONTROL_PLANE.md)) |
| `ble_core` (`include/ble_core.h`, `src/ble_core.cpp`) | Radio bring-up on the core chosen by `TG_BLE_CORE` (Pico 2 W) |

The serial CLI shares `control_api` and is built only with `TG_SERIAL_CLI=1` (H7). Without it the CDC port stays for upload auto-reset and carries no CLI. Enable USB HID with `-DCFG_TUD_HID=1` ([`platformio.ini`](../platformio.ini)).

---

## 10. Implementation phases

| Phase | Scope | Status |
|---|---|---|
| **H0** | Spec + TS types + WebHID docs | done |
| **H1** | HID interface; framing; `hello` / `ping` / `getState` / `getMetrics` | done |
| **H2** | `patchState`, events/errors ops, enable/set parity | done |
| **H3** | `subscribe` pushes + rate caps; structured error notifies | done |
| **H4** | Browser smoke page [`hid/example.html`](hid/example.html) | done (host A/B soak still manual) |
| **H5** | Quartz timing: `getState.quartz`, `getQuartz`, `getQuartzPulses`, `quartzRestart`, `quartz` / `quartzPulse` notifies, example page panel | done (additive; `v` stays 1) |
| **H6** | Device mode, beats, pipeline, GPS detail, timebase log: `getDevice` / `setDevice` / `start` / `stop`, `getBeats` + `subscribe.beats`, `caps.pipeline`, `getGps`, `getTimebaseLog` / `clearTimebaseLog`, `setTimebase.logDeltaPpm`, new `hello.caps` keys, example page panel | done (additive; `v` stays 1) |
| **H7** | BLE transport on Pico 2 W ([`BLE_CONTROL_PLANE.md`](BLE_CONTROL_PLANE.md)), control ownership (§4.6: `claim`, `getLink`, `getDiag`, `linkBusy`, `controlReleased`, `hello.link` / `hello.control`), serial CLI retired behind `TG_SERIAL_CLI` (CDC port kept) | in progress (additive; `v` stays 1) |

---

## 11. Out of scope

- Audio over HID / WebHID
- Replacing CDC
- Protobuf / CBOR wire bodies
- Authenticated or encrypted HID
- Feature-report-based RPC
- Phase-4 STFT spectral denoising — extend via new `blocks` keys and `hello.caps` when added

---

## 12. Host artifacts

| File | Purpose |
|---|---|
| [`HID_CONTROL_PLANE.md`](HID_CONTROL_PLANE.md) | Normative protocol (this doc) |
| [`hid/timegrapher_hid.d.ts`](hid/timegrapher_hid.d.ts) | TypeScript contracts for web apps |
| [`hid/webhid_example.md`](hid/webhid_example.md) | Connect / framer / getState / patchState sketch, device mode, beats, GPS, timebase log |
| [`hid/example.html`](hid/example.html) | Smoke-test page for every op |
| [`BLE_CONTROL_PLANE.md`](BLE_CONTROL_PLANE.md) | BLE transport: GATT layout, frames on BLE, host checklist (uses the frames of this document) |

## Appendix A. CLI → HID parity (H7)

The serial CLI is built only with `TG_SERIAL_CLI=1` (the `*_cli` envs, see [`build_and_flash.md`](build_and_flash.md)). Shipped builds have no CLI, so every CLI function a user may still need must have an HID (or BLE) equivalent, or a stated reason why it does not. The CDC port itself stays in every build (upload auto-reset), and it needs no op.

| CLI command | Function | HID / BLE equivalent |
|---|---|---|
| `help` | Command list | Dropped: CLI-only; the op table is §5 |
| `version` | Firmware, board, source | `hello` (§5.1: `fw`, `board`, `source`, `caps`) |
| `status` | USB, stream and DSP summary | `getDevice` (§5.5), `getMetrics`, `getState` (§6); link state in `getLink` (§5.10) |
| `time` | Device µs timer (calibration probe) | `getTime` |
| `timebase` (show) | Calibration, PPS, GPS summary | `getTimebase` (§5.2) |
| `timebase set <ppm> [src] [date] [dur]` | Apply a calibration (RAM) | `setTimebase` |
| `timebase save` / `clear` | Write or clear the calibration | `saveTimebase` / `clearTimebase` |
| `timebase gps` | NMEA / GPS status | `getGps` (§5.7) |
| `timebase log [clear]` | Calibration history | `getTimebaseLog` / `clearTimebaseLog` (§5.8) |
| `timebase logdelta [ppm]` | Log threshold | `setTimebase` with `logDeltaPpm` (§5.9); read in `getTimebaseLog` |
| `device [mode usbmic\|standalone]` | Operating mode | `getDevice` / `setDevice` (§5.5) |
| `run [start\|stop\|status]` | Run latch | `start` / `stop` / `getDevice` (§5.5) |
| `display` (status, I²C bus scan) | OLED health report | Dropped: bench wiring check. Use a `*_cli` build. Not a user function |
| `display contrast <1-255>` | OLED contrast (runtime, not persisted) | Dropped: cosmetic, resets at boot; the default is the build-time `DISPLAY_CONTRAST`. Reported to the manager as a gap decision |
| `display period <ms>` | OLED refresh period (runtime, not persisted) | Dropped: same reason (`g_period_req` in `display_ui.cpp`) |
| `diag [on\|off]` | Periodic counter dump | `getDiag` (§5.10, A2): the counters are always available on demand; the periodic print is `TG_SERIAL_CLI` only |
| `dsp list` / `status` / `bypass` / `on` / `off` | Graph and block state | `getState` / `patchState` (§6) |
| `dsp get` / `set` | Block parameters | `getState` / `patchState` (§6.3 alias table) |
| `dsp events` / `errors` | Event and error history | `getEvents` / `getErrors` / `clearErrors`, `event` notify (§7) |
| `dsp beat` / `beats` | Beat rate, per-beat list | `getBeat` / `getBeats` (§5.6) |
| `dsp graph` | Block list print | Dropped: debug print; the graph is in `getState` |
| `dsp metrics` | Level and gain metrics | `getMetrics` |
| `dsp timing` | CPU headroom | `getTiming` / `resetTiming` |
| `dsp spectrum` | Spectrum | `getSpectrum` |
| `quartz status` | Quartz snapshot | `getQuartz` (§5.3, §8.5) |
| `quartz mode` / `gate` / `fork` / `ts` / `ksigma` | Quartz settings | `patchState` (§6) |
| `quartz restart` | New measurement | `quartzRestart` |
| `quartz pulses` | Pulse list | `getQuartzPulses` (§8.6) |
| `quartz inhibition` / `jumpers` | Static help text (inhibition, jumper settings) | Dropped: documentation, not a function |
| `ble` (A3, `TG_BLE` + `TG_SERIAL_CLI`) | BLE status, on/off | `getLink` (§5.10); `ble off` has no op (CLI-only maintenance switch) |
| (upload reset, 1200 baud) | Reboot into BOOTSEL | None needed: handled by the CDC driver in every build (F-18) |

Gaps closed by H7: `getDiag` and `getLink` (A2). Open decision for the manager: the three OLED rows marked dropped.
