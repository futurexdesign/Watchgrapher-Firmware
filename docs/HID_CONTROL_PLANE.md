# HID control plane (WebHID / JS-first)

**Status:** Implemented (H1–H3 in firmware; H4 example page). CDC CLI remains available alongside HID. This document is normative for hosts and firmware.

**Goal:** Expose the same DSP surface as [`control_api`](../include/control_api.h) to a browser via [WebHID](https://developer.mozilla.org/en-US/docs/Web/API/WebHID_API), without parsing CLI text or C enums. Audio stays on UAC2; HID carries control JSON only.

**Related:** TypeScript types in [`hid/timegrapher_hid.d.ts`](hid/timegrapher_hid.d.ts), browser sketch in [`hid/webhid_example.md`](hid/webhid_example.md), smoke page [`hid/example.html`](hid/example.html), as-built overview in [`AS_BUILT_ARCHITECTURE.md`](AS_BUILT_ARCHITECTURE.md).

---

## 1. Design summary

| Choice | Decision |
|---|---|
| USB composite | Keep UAC2 mic + CDC CLI; **add** vendor HID (`CFG_TUD_HID=1`) |
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
| 6 | N | UTF-8 chunk (`payloadLen` bytes); remainder of the 64-byte report is unused/zero |

Maximum payload bytes per report after the 6-byte header: **58** (`64 - 1 - 5` if counting report ID separately from the 63-byte WebHID buffer — see §3.2).

**Normative numbers for implementers:**

- Full USB report length: **64** bytes (`reportId` + 63 bytes).
- Header after `reportId`: **5** bytes (`flags` … `payloadLen`).
- Max chunk: **58** bytes.
- Max reassembled JSON: **2048** bytes. Larger messages → device rejects with `error.code = "tooLarge"` when the first fragment declares an impossible `fragCount`, or drops mid-reassembly.
- Reassembly timeout: **500 ms** from first fragment; incomplete messages are discarded.

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

---

## 4. JSON message envelope

Protocol version field: `"v": 1`. Bump only on breaking changes. Unknown `op` → `error.code = "unknownOp"`.

### 4.1 Request (host → device)

```json
{ "v": 1, "id": 7, "op": "getState" }
```

Optional body fields sit alongside `op` (e.g. `patch` for `patchState`, `limit` for `getEvents`).

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
  "error": { "code": "badParam", "message": "notch hz must be 50 or 60" }
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
| `unknownOp` | `op` not recognized |
| `badRequest` | Missing/invalid envelope fields |
| `badParam` | Param name/value rejected by graph |
| `notFound` | Unknown block name |
| `tooLarge` | Reassembled JSON > 2048 or absurd `fragCount` |
| `busy` | Prior request still processing (should be rare) |
| `internal` | Unexpected firmware failure |

---

## 5. Operations

All ops map through `control_api`. CLI string commands (`dsp set …`) are **not** exposed on HID.

| `op` | Body fields | `result` |
|---|---|---|
| `hello` | — | Device identity + caps (§5.1) |
| `ping` | optional `t` (host timestamp) | `{ "t": <echo>, "uptimeMs": N }` |
| `getState` | — | Full `DspState` (§6) |
| `patchState` | `patch`: partial `DspState` | Post-patch full `DspState` |
| `getMetrics` | — | `DspMetrics` |
| `getTiming` | — | `DspTiming` (DSP block + main-loop CPU headroom) |
| `resetTiming` | — | `{ "cleared": true }` |
| `getBeat` | — | `DspBeatRate` |
| `getEvents` | optional `limit` (1..32, default 8) | `{ "events": DspEvent[] }` newest first |
| `getErrors` | — | `{ "errors": DspError[] }` newest first |
| `clearErrors` | — | `{ "cleared": true }` |
| `subscribe` | rates/flags (§7) | Echo of active subscription |

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
    "blocks": [
      "levelMeter", "gain", "limiter", "filterBank",
      "eventDetect", "beatRate", "noiseFloor", "eventLog",
      "softGate", "notch", "agc", "matchedFilter", "firBank", "spectrum"
    ]
  }
}
```

`caps.blocks` lists JSON camelCase keys available in `getState` / `patchState`. `caps.timing` / `timingHzMax` advertise CPU headroom telemetry (`getTiming` / `subscribe.timingHz`).

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
    "beatRate": { "enabled": true, "params": { "liftAngle": 51, "vph": 0 } },
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
    }
  }
}
```

Notes:

- `bypass` is the master process-path gate (analysis may still run when `true`).
- `agc.params.gainDb` is **read-only** telemetry of the live AGC gain; `patchState` ignores writes to it.
- Enabling a process block clears bypass (same rule as CLI / `control_api_set_block_enabled`).

### 6.2 Merge-patch semantics

`patchState` applies a **JSON Merge Patch**-style object:

- Omitted keys → unchanged.
- Present `bypass` → set.
- Present `blocks.<name>.enabled` → enable/disable that block.
- Present `blocks.<name>.params.<key>` → set that param.
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
| `liftAngle` | `liftAngle` / `lift_angle` |
| `vph` | `vph` (`0` = auto nearest A/h) |

`filterBank.preset`: `0`=bypass/off, `1`=wide, `2`=medium, `3`=narrow.  
`notch.hz`: `50` or `60` only.  
`beatRate.liftAngle`: degrees, 10..90 (default 51).

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
  "events": true,
  "errors": true
}
```

| Field | Meaning | Caps |
|---|---|---|
| `metricsHz` | Periodic `notify` / `op: "metrics"` | `0` = off; max **20** |
| `beatHz` | Periodic `notify` / `op: "beat"` | `0` = off; max **5** |
| `timingHz` | Periodic `notify` / `op: "timing"` | `0` = off; max **5** |
| `events` | Push when new analysis events land | boolean; batch ≤ 8 per notify |
| `errors` | Push when new faults / auto-ejects land | boolean |

Omitted subscribe fields leave the previous value unchanged. Response `result` echoes the full active subscription object.

### 7.2 Notify payloads

**metrics** — same fields as `getMetrics` / `DspMetrics`.

**timing** — same fields as `getTiming` / `DspTiming` (`dspBlockUs*`, `loopBusyPct`, underrun/short/ring proxies).

**beat** — same fields as `getBeat` / `DspBeatRate`.

**event** — one or more new events:

```json
{
  "v": 1,
  "notify": true,
  "op": "event",
  "data": { "events": [ /* DspEvent */ ] }
}
```

**error** — structured fault (also covers auto-eject; no CLI text dependency):

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
`beatRate.params.liftAngle` (default 51) and `beatRate.params.vph` (`0` = auto) are set via `patchState`.

Top-level `rateSDay` / `beatErrorMs` / `amplitudeDeg` are the **instant** view (last tick set). Nested windows:

| Key | Meaning |
|---|---|
| `instant` | Same values as the top-level rate/BE/amp fields |
| `avg1s` | Arithmetic mean of instant samples over the last ~1 s |
| `avg10s` | Arithmetic mean of instant samples over the last ~10 s |

Rolling windows need ≥2 contributing samples per metric before `*Valid` is true. `samples` is the contributing tick count (max of the three metric counts).

```json
{
  "vphRaw": 18012.5,
  "vphNearest": 18000,
  "confidence": 0.95,
  "locked": true,
  "intervalSamples": 4800,
  "eventsSeen": 12,
  "intervalsHeld": 8,
  "sourceHz": 24000,
  "rateSDay": 4.2,
  "beatErrorMs": 0.35,
  "amplitudeDeg": 288.0,
  "liftAngleDeg": 51,
  "liftTimeMs": 3.8,
  "rateValid": true,
  "beatErrorValid": true,
  "amplitudeValid": true,
  "instant": {
    "rateSDay": 4.2,
    "beatErrorMs": 0.35,
    "amplitudeDeg": 288.0,
    "samples": 1,
    "rateValid": true,
    "beatErrorValid": true,
    "amplitudeValid": true
  },
  "avg1s": {
    "rateSDay": 3.9,
    "beatErrorMs": 0.32,
    "amplitudeDeg": 286.5,
    "samples": 8,
    "rateValid": true,
    "beatErrorValid": true,
    "amplitudeValid": true
  },
  "avg10s": {
    "rateSDay": 3.7,
    "beatErrorMs": 0.30,
    "amplitudeDeg": 285.0,
    "samples": 72,
    "rateValid": true,
    "beatErrorValid": true,
    "amplitudeValid": true
  }
}
```

### 8.3 `DspEvent`

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

---

## 9. Firmware architecture

| Module | Role |
|---|---|
| [`hid_control.cpp`](../src/hid_control.cpp) / [`hid_control.h`](../include/hid_control.h) | Framer, JSON encode/decode, op dispatch → `control_api_*` |
| `Adafruit_USBD_HID` in `hid_control_usb_begin()` | Vendor report descriptor (Usage Page `0xFFA0`), Input+Output EP |
| [`usb_audio_init`](../src/usb_audio.cpp) | Calls `hid_control_usb_begin()` before re-enumeration |
| [`TimegrapherStand_main`](../src/TimegrapherStand_main.cpp) | `hid_control_task()` each loop |

JSON on device: fixed-schema decoder for known keys; encoder via `snprintf`. One inbound reassembly buffer; TX fragments pumped when `hid.ready()`.

CDC CLI remains fully supported and shares `control_api`. Enable with `-DCFG_TUD_HID=1` ([`platformio.ini`](../platformio.ini)).

---

## 10. Implementation phases

| Phase | Scope | Status |
|---|---|---|
| **H0** | Spec + TS types + WebHID docs | done |
| **H1** | HID interface; framing; `hello` / `ping` / `getState` / `getMetrics` | done |
| **H2** | `patchState`, events/errors ops, enable/set parity | done |
| **H3** | `subscribe` pushes + rate caps; structured error notifies | done |
| **H4** | Browser smoke page [`hid/example.html`](hid/example.html) | done (host A/B soak still manual) |

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
| [`hid/webhid_example.md`](hid/webhid_example.md) | Connect / framer / getState / patchState sketch |
