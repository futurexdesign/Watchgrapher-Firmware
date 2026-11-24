# BLE control plane (Pico 2 W)

**Status:** In progress. Specified here; firmware (`TG_BLE=1` builds, Pico 2 W only) and the web client are being implemented under [`plans/BLE_TRANSPORT_PLAN.md`](plans/BLE_TRANSPORT_PLAN.md). Where this document and the code differ, the code (`src/ble_control.cpp`, `src/control/`) wins once it exists; please report the difference.

**Goal:** A second control transport next to USB HID (WebHID). The web app and any other host can drive the stand from iPad/iPhone (Bluefy), desktop Chrome/Edge and Android Chrome over Web Bluetooth. Audio stays on UAC2 over USB; BLE carries control JSON only.

**Related:** [`HID_CONTROL_PLANE.md`](HID_CONTROL_PLANE.md) (the JSON protocol, the framing and the ops, all unchanged), [`hid/timegrapher_hid.d.ts`](hid/timegrapher_hid.d.ts) (TypeScript types and BLE UUID constants), [`build_and_flash.md`](build_and_flash.md) (Pico 2 W envs and the BLE CLI command), the implementation plan [`plans/BLE_TRANSPORT_PLAN.md`](plans/BLE_TRANSPORT_PLAN.md) and its test spec [`plans/BLE_TRANSPORT_TESTS.md`](plans/BLE_TRANSPORT_TESTS.md).

Tags such as `F-7`, `R-8` and `HW-05` refer to the plan and its test spec, not to this document.

---

## 1. Overview

| Choice | Decision |
|---|---|
| Transport | GATT service with one RX characteristic (host → device), one TX characteristic (device → host, notify) and one INFO characteristic (read) |
| Wire body | The same JSON as HID (`v: 1`, camelCase, `id` / `op` envelope, notifications with `notify: true`) |
| Framing | The same 5-byte header and 58-byte chunk as HID, **one frame per ATT value**, no padding (§4.3) |
| Control | **One link owns control at a time**: USB HID or BLE (§4.7). The other link can still say `hello`, `ping` and `claim` |
| Security | None in v1 (no pairing, bonding or PIN) |
| Hardware | Raspberry Pi Pico 2 W only (CYW43439 radio) |

Limits of v1:

- Audio is never sent over BLE. The audio path (UAC2 over USB) does not change.
- One BLE central at a time (the library supports one connection).
- No pairing, bonding or encryption.
- The browser must stay in the foreground (Web Bluetooth and Bluefy have no background mode).
- BLE is not built for the Kiwi / RP2354 boards (no radio).

---

## 2. Host checklist

Use this list when writing a BLE host (web app, script or native tool). Each item is a requirement unless it says otherwise.

1. **Discover.** Scan for the service UUID `c137e7c0-92b4-4d9e-87e0-9ba757169016`, or for a local name starting with `TG-` (§4.2). Show the name, `TG-XXXX`, so the user can tell several stands apart.
2. **Connect and discover.** Connect, then get the service and its three characteristics (RX, TX, INFO) by UUID. Use the UUIDs exactly as listed in §4.1; they are lowercase.
3. **Read INFO.** Parse the JSON (`p`, `mtu`, `fmax`, `seg`). Reject a `p` other than `1`.
4. **Request an MTU of at least 66.** Only the client can start the MTU exchange (§4.5). A frame of 63 bytes needs an ATT MTU of 66. Read INFO again after the exchange, because `mtu` and `fmax` change.
5. **Pick the mode from INFO.** `seg: 0` (the normal case): one frame per write, frames up to `fmax` bytes. `seg: 1` (only when the client negotiated an MTU below 66 and the device is built with segmented mode): use the segmented mode of §4.5.
6. **Subscribe before writing.** Call `startNotifications()` on TX and wait for it to finish **before** the first RX write.
7. **One frame per write.** Build each frame as `flags, msgSeq, fragIndex, fragCount, payloadLen, chunk` with the length `5 + payloadLen`. Do not pad to 63 bytes (the device accepts padded writes but does not need them). Never write more than 63 bytes.
8. **Write sequentially.** Send the fragments of one request in order, and wait for each GATT write to complete before the next. Use `writeValueWithResponse` unless the hardware test HW-07 chose `writeValueWithoutResponse`.
9. **Keep one request in flight.** Send the next request only after the reply (or a timeout) of the previous one. Use a request timeout of 2 to 4 s.
10. **Keep `msgSeq` per link.** The HID and BLE sessions number their messages independently. Use a fresh `msgSeq` for every message (the counter wraps at 256). Never resend the fragments of a completed request.
11. **Reassemble.** Key reassembly by `msgSeq`. A new `msgSeq` discards the incomplete message. Discard the partial message after 500 ms without the next fragment. Drop frames whose `fragCount` is 0, whose `fragIndex` is not below `fragCount`, or whose `payloadLen` is above 58.
12. **Keep the lease alive.** The device gives the control lease 10 s. A host that owns control and sends no request for 3 s must send `ping`. Otherwise another link can take control.
13. **Check ownership.** After `hello`, read `control`. If `owner` is set and `self` is false, do not proceed with control. Offer the user a takeover: send `claim` with `takeover: true`.
14. **Handle `linkBusy`.** A request refused with `linkBusy` was not run. Show that another link holds control, and offer a takeover.
15. **Handle `controlReleased`.** Stop your subscriptions and show that control moved to the `to` link. Do not send further subscription requests unless the user takes control again.
16. **Handle disconnects.** On disconnect, drop the partial reassembly, fail the pending request and clear your subscription state. The device clears its subscriptions too (§4.4). After reconnecting, send `hello` and read `control` again.
17. **Respect the size limit.** Do not send a request body larger than 2030 bytes (35 fragments of 58 bytes), as for HID (HID_CONTROL_PLANE.md §3.4).

---

## 3. Frames and ops

BLE uses the frames and JSON of [`HID_CONTROL_PLANE.md`](HID_CONTROL_PLANE.md) §3 and §4. The differences are the ones in §4.3 below (no report ID, no padding, one frame per ATT value) and the ownership rules of §4.7. The ops, their results and the notifications are the same on both links, plus the additions of §4.6 (`claim`, `getLink`, `getDiag`, `controlReleased`, `linkBusy`, and the `hello` keys).

---

## 4. BLE control-plane specification (normative)

### 4.1 GATT layout

All UUIDs are lowercase (Web Bluetooth requires it). Base generated 2026-11-22 with `uuidgen`.

| Item | UUID | Properties | Value |
|---|---|---|---|
| **Service** "Timegrapher Control" | `c137e7c0-92b4-4d9e-87e0-9ba757169016` | primary | — |
| **RX** (host → device) | `c137e7c1-92b4-4d9e-87e0-9ba757169016` | `Write`, `WriteWithoutResponse` | One frame (§4.3) per ATT write |
| **TX** (device → host) | `c137e7c2-92b4-4d9e-87e0-9ba757169016` | `Notify` | One frame per notification |
| **INFO** | `c137e7c3-92b4-4d9e-87e0-9ba757169016` | `Read` | UTF-8 JSON, ≤ 64 bytes: `{"p":1,"mtu":185,"fmax":63,"seg":0,"txMax":244}` |

INFO fields:

| Key | Meaning |
|---|---|
| `p` | BLE link-layer version, **1**. |
| `mtu` | ATT MTU currently negotiated (23 until the exchange completes). |
| `fmax` | Largest frame the device sends or accepts without segmentation: `min(63, mtu − 3)`. |
| `txMax` | Largest device-to-host frame the device will use (244 on the Pico 2 W; a larger `frameMax` is lowered to it). Absent on firmware without large frames. See §4.3a. |
| `seg` | `0` = one frame per ATT value (§4.3). `1` = segmented mode (§4.5, fallback only). |

The INFO value is rebuilt **in BT context** in the MTU-exchange event and on connect, and is never written from the main loop (F-7).

### 4.2 Advertising and discovery

- Connectable undirected advertising. Interval 100–150 ms while unconnected (library default if close), and stopped while connected (one connection, F-11).
- Payload, which must fit 31 bytes: Flags (3) + 128-bit service UUID (18) + Complete Local Name `TG-XXXX` (2 + 7 = 9) = **30 bytes**.
- `XXXX` is the last two bytes of the BD_ADDR in upper-case hex (e.g. `TG-1A2B`). The OLED and the `ble` CLI command show the same string, so a user can tell several stands apart.
- Web host filter: `requestDevice({ filters: [{ services: [SERVICE] }, { namePrefix: 'TG-' }], optionalServices: [SERVICE] })`. The `namePrefix` alternative covers browser apps that filter only on scan-response data.
- Security: `BLESecurityNone` (v1).

### 4.3 Frames on BLE

- The **same** frame as [HID_CONTROL_PLANE.md](HID_CONTROL_PLANE.md) §3.2 (the 63-byte WebHID buffer without a report ID): `flags, msgSeq, fragIndex, fragCount, payloadLen, chunk[payloadLen ≤ 58]`. All §3.3/§3.4 rules apply unchanged, including the 58-byte chunk rule, the 2048-byte message cap, at most 35 request fragments, the 500 ms reassembly timeout and one request at a time.
- **No padding on BLE**: the ATT value length is exactly `5 + payloadLen` (5..63). The device also accepts padded 63-byte writes, and ignores bytes after `5 + payloadLen`.
- **One frame per ATT value.** A value never carries two frames or part of one (unless `seg:1`, §4.5).
- A host write longer than 63 bytes is dropped as invalid.
- The host sends a request's frames **sequentially**, awaiting each GATT write before the next. Default `writeValueWithResponse`; `writeValueWithoutResponse` is optional and chosen in HW-07.
- The host must `startNotifications()` on TX **before** its first RX write.
- `msgSeq` counters are per link: the HID and BLE sessions number their messages independently.

### 4.3a Large device-to-host frames (opt-in)

The frame format is unchanged (`payloadLen` is a u8), but the 58-byte chunk limit applies only to frames **from the host**. A host that reads `txMax` in INFO may send `hello {"frameMax": N}` (any `hello`; absent keeps the current value). After the reply to that `hello`, every device-to-host frame on the link has at most `txFrame = min(N, txMax, ATT MTU − 3)` bytes (`txMax` is 244 on the Pico 2 W, so a chunk of up to 239 bytes; the protocol itself allows 255). Larger notifications (an HCI ACL payload above 255 bytes) stall the CYW43 radio on this stack, measured 2026-11-23.

- The `hello` reply itself still uses 63-byte frames. Its `link.frameMax` reports the value in use for later messages.
- A `frameMax` below 63 is ignored. The size resets to 63 when the link drops. Hosts that never send `frameMax` never see a frame above 63 bytes.
- A frame never exceeds the ATT payload: if the MTU shrinks, the next message uses the smaller size.
- Writes to RX stay at most 63 bytes. USB HID ignores `frameMax` and reports 63.
- With large frames, a receiver accepts `payloadLen` up to 250 in non-final fragments, and fragment `i` starts at the sum of the earlier chunk lengths (not `i × 58`).

### 4.4 Connection behaviour

| Event | Device behaviour |
|---|---|
| Connect | Store `con_handle`; rebuild INFO; ~1 s after connect, request a connection interval of **exactly 15 ms** (`min = max = 12` in 1.25 ms units), latency 0, supervision timeout 4 s (Apple accessory guidelines allow it). If the central rejects the request (`L2CAP_EVENT_CONNECTION_PARAMETER_UPDATE_RESPONSE` with a non-zero result), the device asks once for **15–30 ms** (`12..24`). A second rejection ends the attempts. A central that accepts and then picks another interval is not asked again. `getLink.ble.connParamStep` (`0` none, `1` asked 15 ms, `2` asked 15–30 ms, `3` gave up), `connParamRejects` and `connIntervalUs` show the outcome. |
| MTU exchange | Store the MTU; rebuild INFO. |
| CCCD enabled on TX | Link becomes `connected()` for the session (pushes allowed). |
| CCCD disabled / disconnect | `Session::on_link_down()`: partial RX discarded, TX aborted, **subscriptions of the BLE handler cleared**, TX ring flushed. Advertising resumes: if BTstack doesn't resume it automatically, the TX worker calls `gap_advertisements_enable(1)` (never `BLE.startAdvertising()` again, F-27). HW-11 verifies. |
| RX ring full | The frame is dropped and `rx_dropped` is counted. The message then times out on the host (safe: §3.4 semantics). |
| TX ring full | `Link::can_send()` returns false, so the session holds the frame (backpressure, nothing lost). |

### 4.5 Fallback: segmented mode (build only if gate G1 needs it)

This is used only when a must-have client negotiates MTU < 66. INFO then reports `seg:1` and `fmax = mtu − 4`.

- Each ATT value = `segHdr` (1 byte) + up to `mtu − 4` frame bytes. `segHdr` bit7 = START (first segment of a frame); bits 0..6 are reserved (0).
- A frame's bytes are split across consecutive values. A value never carries bytes of two frames.
- Receiver: on START, reset the accumulator, then append. Once the accumulator holds ≥ 5 bytes and ≥ 5 + payloadLen, the frame is complete. A non-START value with an empty accumulator is dropped.
- Hosts read INFO after connecting and pick the mode. In segmented mode, host writes use `mtu − 4`-byte segments.

Test cases `SEG-*` exist in the test spec, marked *conditional*.

### 4.6 Protocol additions (additive; `v` and `protocolVersion` stay 1)

| Where | Addition |
|---|---|
| `hello.link` (new object) | `{ "kind": "usbHid" \| "ble", "mtu"?: number, "frameMax"?: number }`. Describes **the link this request arrived on**; `mtu` is present only for BLE. `frameMax` is the TX frame size in bytes that applies to messages after this reply: 63 unless the host opted in (§4.3a). |
| `hello.control` (new object) | `{ "owner": "usbHid" \| "ble" \| null, "self": boolean, "leaseMs": 10000 }`. Who holds control right now (§4.7). |
| `hello.caps.ble` | `true` when the firmware is built with BLE (`TG_BLE=1`), absent otherwise. |
| `hello.caps.cli` | `true` when the serial CLI is built (`TG_SERIAL_CLI=1`), absent otherwise (§5.7). |
| `hello.board` | `"Pico2W"` on W builds (`TG_BOARD_NAME`). |
| Op `claim` | Request `{ "takeover"?: boolean }`; result `{ "owner": "<kind>", "previous": "<kind>" \| null }`. §4.7. |
| Op `getLink` | Result `{ "usbHid": { "mounted": bool }, "ble"?: { "enabled", "name", "address", "advertising", "connected", "mtu", "connIntervalUs", "connParamStep", "connParamRejects", "rxFrames", "txFrames", "rxDropped", "rxInvalid", "txRetries", "disconnects", "lastDisconnectReason" }, "control": <as hello.control> }`. Replaces the planned `ble` CLI command for non-CLI builds. Owner-only under §4.7: a non-owner gets `linkBusy` while another link holds a live lease, otherwise the request auto-claims. `hello.control` reads the owner without claiming. |
| Op `getDiag` | Result: the `diag_counters_t` counters (`dmaOverrunCount`, `pcmRingUnderrunCount`, `pcmRingOverrunCount`, `pcm1808DmaBlockOverruns`, …, camelCase of the struct fields). Replaces CLI `diag` for HW tests. Same ownership rule as `getLink` (§4.7). |
| Error `linkBusy` | A request from a non-owner link while another link holds control. `message`: `"control held by usbHid"` / `"… by ble"`. |
| Notify `controlReleased` | `{ "v":1, "notify":true, "op":"controlReleased", "data": { "to": "<kind>" } }` sent to the **previous** owner when another link takes over (if it's still connected). |

Additional fields of `getLink.ble` (plan §5.8): `core` (`0` or `1`, the core that runs the radio; `1` in every shipped W build, `0` only with the `TG_BLE_CORE=0` override in `docs/build_and_flash.md`) and, when `core` is `1`, `core1StackFree` (free bytes of core 1's stack, high-water mark).

The TypeScript types are in [`hid/timegrapher_hid.d.ts`](hid/timegrapher_hid.d.ts) (`HidHelloLink`, `HidHelloControl`, `HidLinkStatus`, `HidDiag`, the `claim` / `getLink` / `getDiag` types, `linkBusy`, `controlReleased`, `HidHelloCaps.ble` / `.cli`). They are synced to the web app with `npm run sync-types`.

### 4.7 Control ownership: one link at a time

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
- Hosts that stay connected without sending requests (the web app on a passive view) must keep the lease alive: the web transports send `ping` after 3 s without any request ([plan §6.2](plans/BLE_TRANSPORT_PLAN.md)). WebHID gives the device no close signal, so without the lease a closed desktop tab would block BLE forever.
- RAM: one `Session` (reassembler + fragmenter, about 4.1 KB) per link, and **one** `ControlHandler`. The shared scratch buffers (`g_out`, `g_patch_obj`, …) stay single file-scope instances.

---

## 5. Hardware walkthrough with nRF Connect

This walkthrough uses a generic BLE client to check the link without the web app. It needs a Pico 2 W BLE build (step 4 needs the echo build, step 5 the protocol build; see [`build_and_flash.md`](build_and_flash.md)) and nRF Connect, or any client that can write hex values and enable notifications.

1. **Scan.** Find `TG-XXXX` in the scan list. The advertisement lists the service `c137e7c0-…`. Connect.
2. **MTU.** Request an MTU of 185 (nRF Connect offers the request in the connected device's menu). Then read INFO (`c137e7c3-…`). Expect `p` = 1 and `mtu` ≥ 66.
3. **Subscribe.** Open TX (`c137e7c2-…`) and enable notifications **before** the first write.
4. **Echo check (G1 build).** Write this 31-byte frame to RX (`c137e7c1-…`) as a byte array in hex. It is the `ping` request `{"v":1,"id":1,"op":"ping"}`:

   ```text
   000100011A7B2276223A312C226964223A312C226F70223A2270696E67227D
   ```

   Byte layout: `00` flags, `01` msgSeq, `00` fragIndex, `01` fragCount, `1A` payloadLen (26), then the 26 JSON bytes. The echo build returns the same 31 bytes as one notification. The protocol build returns a `ping` reply whose JSON contains `"id":1,"ok":true`.

5. **Protocol check (protocol build).** Write the `hello` frame, 32 bytes, with `msgSeq` 2:

   ```text
   000200011B7B2276223A312C226964223A322C226F70223A2268656C6C6F227D
   ```

   It is `{"v":1,"id":2,"op":"hello"}` (27 bytes, `1B` payloadLen). The reply is a notification with `"id":2` and a `result` containing `link.kind` = `"ble"` and `control`.

6. **Disconnect.** Disconnect and check that advertising resumes (the name is visible again within a few seconds).

---

## 6. Build and test

- Envs, the `ble` CLI command and the echo build override are in [`build_and_flash.md`](build_and_flash.md) ("Pico 2 W (BLE)").
- The acceptance tests are in [`plans/BLE_TRANSPORT_TESTS.md`](plans/BLE_TRANSPORT_TESTS.md). Host tests for the framing, sessions and ownership run without hardware (`tests/run_all.sh`).
