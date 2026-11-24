# Plan: BLE control-plane transport (Pico 2 W) + Web Bluetooth client

| | |
|---|---|
| **Status** | Approved for implementation — 2026-11-22 |
| **Hardware** | Raspberry Pi Pico 2 W boards arrive 2026-11-23 (hardware day, §9) |
| **Repos** | `timegrapher-stand` (this repo, firmware + protocol) and sibling `timegrapher-web` (web app) |
| **Tests** | Every test ID used here (`FR-*`, `SES-*`, `WB-*`, `HW-*` …) is specified in [`BLE_TRANSPORT_TESTS.md`](BLE_TRANSPORT_TESTS.md) |
| **Workflow** | Opus manager splits work into the sub-agent tasks in §8, reviews each submission, kicks back with findings, then verifies builds/tests itself. **Nobody commits**: all work stays in the working tree; the user commits. |
| **Revision 2** | 2026-11-22, user decisions: (1) **only one of USB HID or BLE controls the device at a time** (§4.7, replaces the concurrent policy); (2) **Bluetooth icon in the OLED header**, styled like the GPS badge (§5.6); (3) **serial CLI retired behind `TG_SERIAL_CLI`**, omitted by default and stripped from the build (§5.7). New tasks A7 (display) and A8 (CLI retirement). |
| **Revision 3** | 2026-11-22, user decisions: (1) **the CDC interface stays in every build**, also without the CLI, so PlatformIO's 1200-baud auto-reset into BOOTSEL keeps working (F-18, F-20, §5.7); the `bootsel` HID op is dropped. (2) The CPU division was reviewed (§5.8): USB, capture, DSP and control stay on core 0; **the radio becomes core-selectable (`TG_BLE_CORE`)**, with core 1 next to the display as the preferred target, chosen on hardware day (HW-19). New task A9. |
| **Revision 4** | 2026-11-23, hardware day: A3-M2 landed (BLE session) and the pacer stall was fixed (PACE-11). **The synthetic source now feeds the DSP graph** (rate / amplitude / beat error with no pickup). New workstream §13: a **detection pre-filter** so noise filtering (notch, band-pass) reaches the beat detector, plus matched-filter fixes, synthetic impairments for testing without a PCM1808, and timing-stat clarity. New tasks A10–A13, W5. |
| **Revision 5** | 2026-11-23, after the HW-07 throughput measurements (≈ 100 notifications/s, 58 JSON bytes each): §14 makes the BLE link fit a 36,000 bph timegrapher with headroom: a **packed `beats` stream** inside the JSON envelope, **opt-in large TX frames** on BLE, and a **15 ms connection interval** with a fallback. All additive: `v` and `protocolVersion` stay 1 and old clients see no change. Tasks A14, A15, W6. |

---

## 1. Goal

Add a second **control-plane transport**, Bluetooth Low Energy, so the existing remotely-served web app (GitHub Pages, HTTPS) can drive the timegrapher from:

- **iPad / iPhone** via a Web Bluetooth browser app such as **Bluefy** (Safari has no WebHID/Web Bluetooth),
- desktop Chrome/Edge and Android Chrome via native **Web Bluetooth**,

while **USB HID (WebHID) keeps working unchanged** on every existing board. **Only one link controls the device at a time** (§4.7): the other link can still say `hello`, sees who holds control, and can take over explicitly. The web app itself only ever has one transport open.

With USB HID and BLE covering every host, the **serial CLI is retired** behind a compile flag (§5.7). The **CDC serial port itself stays** in every build, so upload auto-reset keeps working.

### 1.1 Non-goals (v1)

- Audio over BLE (audio stays UAC2 over USB; nothing about the audio path changes).
- Wi-Fi / WebSocket transport (the refactor in §5 makes it a later drop-in, but it is not built now).
- BLE pairing, bonding, encryption or an app-level PIN (§11 risk R-8).
- BLE on the Kiwi / RP2354 boards (no radio).
- Changing the JSON protocol: all ops, payloads and the 5-byte frame header stay as in `docs/HID_CONTROL_PLANE.md`. Only additive `hello` keys are added (§4.6).

### 1.2 Success criteria

1. The `pico2w_*` firmware builds in CI. On hardware, an iPad running Bluefy connects to the GitHub Pages web app over BLE and every view works (HW-08).
2. USB audio is not degraded by BLE: zero ring underruns/overruns in a 1 h soak with USB audio streaming plus a subscribed BLE session (HW-06, HW-13).
3. All existing host tests, web unit tests and the transport contract suite still pass. The contract suite also passes over the new `WebBleTransport` against the simulator (CT-BLE-01).
4. Existing non-W firmware builds behave the same as before over USB HID (regression HW-01, SES tests). With `TG_SERIAL_CLI` omitted they are **smaller** than today (the CLI is about 49.7 KB of flash and 1.8 KB of RAM; BLD-01, CLI-01).
5. Control is exclusive: while one link holds control, the other link's requests get `linkBusy`, and an explicit takeover works from the web app (ARB-*, HW-10).
6. The OLED shows the Bluetooth badge, hollow while not connected and inverted while connected (UIH-*, HW-17).

---

## 2. Facts established before planning (verified 2026-11-22)

These were checked against the installed toolchain (arduino-pico **5.5.1**, `framework-arduinopico 1.50501.0+sha.275e73d`, platform pinned at `79ce473`). Agents must not re-litigate them; if one looks wrong in practice, report it to the manager.

| # | Fact | Consequence |
|---|---|---|
| F-1 | `-DPIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH` makes the platform link `liblwip-bt.a` and define `ENABLE_BLE`, `ENABLE_CLASSIC`, `CYW43_ENABLE_BLUETOOTH` (`tools/platformio-build.py:87`). | Build flag for the W envs. |
| F-2 | A spike build (current firmware with a BLE GATT service and Adafruit TinyUSB) **compiles and links**: `BLEServer::addService`, `att_server_get_mtu`, `tud_task_ext` and `Adafruit_USBD_HID::begin` are all in one image. RAM 164.6 KB → **244.0 KB** (46.5 %), flash 255 KB → **659 KB** (15.7 % of 4 MB). | Fits. *Runtime* coexistence is still unproven and is hardware gate G1. |
| F-3 | Board `rpipico2w` exists in the pinned platform. | `board = rpipico2w`. |
| F-4 | Pico 2 W radio pins: GPIO23 (WL_ON), GPIO24 (WL_D), **GPIO25 (WL_CS)**, GPIO29 (WL_CLK / VSYS ADC). `include/board_config.h` sets `STATUS_LED_GPIO 25` for `BOARD_PICO2`. | **The W build must not drive GPIO25.** The status LED goes through the CYW43 (`LED_BUILTIN`). Never use ADC on GPIO29. |
| F-5 | The project's Pico 2 pins (3, 4, 5, 18–21, 26) do not collide with the radio pins. The PCM1808 capture hard-codes `pio0`, the PPS counter and CYW43 claim free state machines, and the RP2350 has 3 PIO blocks. | No pin changes. PIO claim order must be checked on hardware (HW-01). |
| F-6 | arduino-pico `BLE` library: GATT server, `BLECharacteristic` (`BLEWrite`, `BLEWriteWithoutResponse`, `BLENotify`, `BLERead`), write and read callbacks run **in BT context (interrupt level, core 0)**. | The RX path must hand frames off to the main loop through a lock-free queue (§5.4). |
| F-7 | `BLECharacteristic::setValue()` `realloc`s the one value buffer that BTstack's later can-send callback reads, and gives no "sent" callback. Calling it again before the notification goes out **loses or corrupts** the previous value. The library's `BLEServiceUART::flush()` has the same flaw. | **Never use `setValue()` on the TX characteristic.** The transport keeps its own frame queue and calls `att_server_notify()` from its own `att_server_register_can_send_now_callback()` registration (§5.4). |
| F-8 | `BLECharacteristic::handleWrite()` ignores `offset` and `transaction_mode`, so ATT long/prepared writes are broken. | Every host write must fit in **one** ATT write of ≤ MTU−3 bytes. |
| F-9 | Only the **client** can start the ATT MTU exchange. BTstack's buffers are large (`HCI_ACL_PAYLOAD_SIZE 1695`), so the client decides the MTU. A 63-byte frame needs **ATT MTU ≥ 66**. | One frame per ATT value needs MTU ≥ 66, which is measured per client in HW-04. A segmentation fallback is specified (§4.5) and built only if gate G1 needs it. |
| F-10 | The firmware RX reassembler places fragment *i* at offset `i × 58`, so every non-final fragment must carry exactly 58 payload bytes. The web `FrameReassembler` concatenates parts by index and accepts frames shorter than 63 bytes (`5 + payloadLen ≤ length`). | Frames may be **unpadded** on BLE (5 + payloadLen bytes). The 58-byte chunk rule is unchanged. |
| F-11 | The library supports **one** BLE connection (`BLEServer::con_handle`). | One BLE central at a time; advertising resumes after disconnect (verified in HW-11). |
| F-12 | `BLEServer::prepareAdvertising()` builds a single 31-byte advertisement with the service UUID, and `Serial.println()`s the service UUID on the CDC port at `startAdvertising()`. | The name must be short: `TG-XXXX` (§4.2). One UUID line on the CLI at boot is harmless; mention it in the docs. |
| F-13 | `BLECharacteristic` exposes `protected` `_valueHandle`, `_configData` (CCCD), `virtual setConHandle()` and `virtual disconnected()`. | A subclass can track the connection handle and the CCCD state without patching the library. |
| F-14 | BTstack pairing storage (`__bluetooth_tlv`) is allocated **inside** the program image (`cores/rp2040/sdkoverride/btstack_flash_bank.cpp`), not in the 4 KB EEPROM sector `flash_store` owns. With `BLESecurityNone` there is no bonding, so no runtime TLV writes are expected. | No flash-layout change. Flash writes while BLE is connected are still tested (HW-14). |
| F-15 | **Pre-existing bug:** `hid_control.cpp tx_pump()` advances `g_tx_off` before `sendReport()` but advances `g_tx_frag_index` only on success. A failed send makes the retry send the *next* chunk under the *old* index, which corrupts the message. | Fixed by the peek/commit fragmenter (§5.2); regression tests FR-TX-07 and SES-05. |
| F-17 | **The BLE stack runs on core 0.** The `rpipico2w` variant's `initVariant()` calls `init_cyw43_wifi()` → `cyw43_arch_init_with_country()` from `main()` on core 0, **before** `setup()` and before core 1 is launched (`cores/rp2040/main.cpp`). The arch uses the *threadsafe-background* async context, which binds the CYW43 driver and BTstack work (and the WL_D GPIO interrupt) to the core that initialised it. BLE callbacks therefore run as **low-priority interrupt work on core 0**, pre-empting `loop()`: the same core as USB, audio capture and DSP. Core 1 runs only the display. This happens on every W build, including `pico2w_synthetic_noble` (the radio chip is brought up even without BLE). | Risk R-1 is the main hardware question. §5.8 makes the radio core selectable (`TG_BLE_CORE`), and HW-19 compares the two. |
| F-18 | The 1200-baud "touch" is handled **inside the Adafruit library** (`Adafruit_USBD_CDC.cpp` `tud_cdc_line_state_cb`: DTR dropped at 1200 baud → `TinyUSB_Port_EnterDFU()`), not by the CLI. It works whenever the CDC interface exists. `Adafruit_USBD_CDC::write()` loops (calling `yield()`) while a terminal holds DTR and the 256-byte TX FIFO is full; with DTR low it drops the data. (`-DCFG_TUD_CDC=0` doesn't compile anyway: `Adafruit_TinyUSB_API.cpp` calls `tud_cdc_n_write_flush()` unconditionally.) | **CDC is kept in every build** (user decision, revision 3). No USB descriptor change, so no-CLI builds don't touch `usb_audio.cpp`. No-CLI builds must not write to `Serial` (an open, unread terminal would stall the loop), apart from the library's one boot line (F-12). |
| F-19 | CLI cost in `pico2_pcm1808` (object sizes): `cli_commands.cpp` 48.0 KB text + 1.7 KB bss, `cli_core.cpp` 1.6 KB, `serial_cli.cpp` 0.1 KB, **≈ 49.7 KB flash, 1.8 KB RAM** in total. Only `diagnostics.cpp` (periodic print) and `TimegrapherStand_main.cpp` (`serial_cli_init/tick`) use it from outside `src/cli/`. `scripts/calibrate_timebase.py` drives the CLI (the web app's `usePcCalibration` replaces it). | §5.7 build filtering; the HID parity audit adds the few missing ops (A8). |
| F-20 | Because CDC stays, `pio run -t upload` keeps auto-resetting a running device into BOOTSEL, with or without the CLI. | No `bootsel` HID op and no `hid_bootsel.py` (dropped in revision 3). HW-18 checks the upload on a no-CLI build. |
| F-21 | The OLED header is full. The mechanical header text can be 14 characters (84 px: `no host stream`, `clock check...`), the mode tag `USB` (18 px) sits at x 85–102 and the GPS pill (23 × 11) at x 105–127. The quartz header `QUARTZ` + `UNCAL` badge ends at x 78. An extra 11 px Bluetooth badge + 2 px gap moves the mode tag to x 72–89, leaving **≤ 70 px (11 characters)** for the left text. | Layout rule and string changes in §5.6 (decision D-UI, §12). |
| F-22 | **Async context across cores** (pico-sdk `async_context_threadsafe_background.c`, `MULTI_CORE` on). `async_context_set_work_pending()` is lock-free and callable from any core or IRQ: from the other core it wakes the context through an alarm on the context core's alarm pool. Releasing the context lock (`BluetoothLock`) **on the context's own core runs all pending BT work inline**, in the caller's thread. Released from the other core, it only wakes the context core. A context initialised on core 1 creates its own alarm pool on a free hardware alarm. | The main loop must not take `BluetoothLock` (on core 0 it would run BTstack work inside `loop()`). It signals BT context with `set_work_pending` instead (§5.8). |
| F-23 | The CYW43 gSPI bus driver (`cyw43_bus_pio_spi.c`) **busy-waits** for its DMA transfers (`dma_channel_wait_for_finish_blocking`). It claims one PIO state machine and two DMA channels, and its host-wake interrupt is `IO_IRQ_BANK0` on the context core. Everything happens in `initVariant()`, **before** `setup()`, so CYW43 claims its PIO SM and DMA channels before PCM1808 capture does. | BT work costs CPU for the length of each SPI transfer, not just for the protocol work. Fixes R-9's claim-order assumption: PCM1808 uses `pio_claim_free_sm_and_add_program` and takes whatever is left. |
| F-24 | The variant's `initVariant()` is linked as an **object file** (`platformio-build.py`: "link variant's source files as object files"), so a sketch can't override it (duplicate symbol). But the builder honours `board_build.variants_dir` / `board_build.variant`, so a **project-local copy of the `rpipico2w` variant** can supply an `initVariant()` that doesn't start the radio. `init_cyw43_wifi()` is `extern "C"` and callable later from any core. | Core-1 radio placement without a double init (§5.8). |
| F-25 | On the Pico 2 W the status LED is a **CYW43 GPIO**. Each `digitalWrite`/`cyw43_arch_gpio_put` takes the async context lock. Today's `led_set()` toggles it at 1–10 Hz from `loop()`. | Under F-22 the LED alone would run BT work inside `loop()` or block on core 1's lock. The LED is driven from BT context (§5.8). |
| F-26 | Core 1 runs with an 8 KB heap stack (`core1_separate_stack`, `main.cpp`: `malloc(0x2000)`). IRQ handlers on core 1 use the same stack. The display loop is idle about 85–90 % of the time: it renders, spends ~55 ms in blocking I²C per frame, and sleeps the rest of each 500 ms period (`display_ui.cpp`). | Room for the radio on core 1, but the stack is shared with BT IRQ work: measure the high-water mark (A9). |
| F-27 | `BLE.startAdvertising()` **rebuilds the whole ATT database** (`att_db_util_init`, `att_server_init`, `prepareAdvertising()`, which also prints the UUID, F-12) every time it is called. | Call it once in `ble_control_begin()`. To resume advertising after a disconnect, call `gap_advertisements_enable(1)` from BT context, never `startAdvertising()` again. |
| F-16 | `timegrapher-web` already has a `Transport` interface (`src/transport/Transport.ts`), `WebHidTransport`, a simulator (`VirtualDevice.handle()` / `pollNotify()`), and a harness-based **transport contract suite** (`tests/contract/`). Deployed to GitHub Pages over HTTPS (secure context: Web Bluetooth OK). | BLE is a third `Transport`. The contract suite runs over it with a fake GATT peripheral (CT-BLE-01). |

---

## 3. Architecture

### 3.1 Firmware (after this work)

```text
                    ┌──────────────────────────── main loop (core 0) ─────────────────────────────────┐
USB OUT report ──►  UsbHidLink ──► Session[HID] ──┐                                                  │
(TinyUSB task,      (adapter)       FrameReassembler│                                                │
 main-loop thread)                  FrameFragmenter │   ┌─ owner ──► ControlHandler (one) ─► control_api
                                                    ├──►│ Arbiter                ▲ pushes to owner only
BLE write (BT ctx) ─► RX FrameRing ─► BleLink ─► Session[BLE]  └─ non-owner ─► GateResponder (hello/ping/claim/linkBusy)
                      (SPSC)         (adapter)                                                       │
BLE notify (BT ctx) ◄─ TX FrameRing ◄┘  TX worker → can-send-now callback pops one frame → att_server_notify│
                    └───────────────────────────────────────────────────────────────────────────────┘
   BT context = low-priority IRQ work on the radio core: core 0 (TG_BLE_CORE=0) or core 1 next to the display (TG_BLE_CORE=1), §5.8
   main loop → BT context only through the rings, atomics and async_context_set_work_pending (no BluetoothLock in loop())
```

- **`control::FrameReassembler` / `control::FrameFragmenter`** (§5.2): pure, host-testable framing extracted from `hid_control.cpp`, with the behaviour of HID_CONTROL_PLANE.md §3.4 unchanged.
- **`control::Link`** (§5.3): abstract transport (can_send / send_frame / pad policy / connected).
- **`control::Session`** (§5.3): one per link. Owns reassembly, fragmentation, one-request-at-a-time dispatch and the push slot. Hands complete requests to the **`control::Arbiter`**.
- **`control::Arbiter`** (§4.7, §5.3): pure, host-testable ownership state machine (none / usbHid / ble, lease, takeover). It routes the owner's requests to the single `ControlHandler`, answers the non-owner's `hello` / `ping` / `claim` itself, and refuses everything else with `linkBusy`.
- **`ControlHandler`** (§5.3): the existing op dispatch, encoders and subscription engine from `hid_control.cpp` behind the `Handler` interface. There is **one instance**; its subscription state can stay file-scope, as today, because only the owner session ever reaches it.
- **`UsbHidLink`** (in `hid_control.cpp`): descriptor and Adafruit HID glue only.
- **`BleLink`** (in `ble_control.cpp`, only with `TG_BLE=1`): GATT service, SPSC rings, can-send pacing, advertising, connection parameters, status.

### 3.2 Web app (after this work)

```text
stores ─► Transport ─┬─ WebHidTransport ─┐
                     ├─ WebBleTransport ─┴─ FramedTransport (shared queue / ids / timeouts / notify / framer)
                     └─ VirtualTransport (simulator)
```

`FramedTransport` (W1) is an abstract base extracted from `WebHidTransport` that holds everything not specific to WebHID. `WebBleTransport` (W2) supplies only "open link / write one frame / frames in / link lost".

---

## 4. BLE control-plane specification (normative for this work)

A5 copies this section into the new `docs/BLE_CONTROL_PLANE.md`. Until then this section is the spec.

### 4.1 GATT layout

All UUIDs are lowercase (Web Bluetooth requires it). Base generated 2026-11-22 with `uuidgen`.

| Item | UUID | Properties | Value |
|---|---|---|---|
| **Service** "Timegrapher Control" | `c137e7c0-92b4-4d9e-87e0-9ba757169016` | primary | — |
| **RX** (host → device) | `c137e7c1-92b4-4d9e-87e0-9ba757169016` | `Write`, `WriteWithoutResponse` | One frame (§4.3) per ATT write |
| **TX** (device → host) | `c137e7c2-92b4-4d9e-87e0-9ba757169016` | `Notify` | One frame per notification |
| **INFO** | `c137e7c3-92b4-4d9e-87e0-9ba757169016` | `Read` | UTF-8 JSON, ≤ 64 bytes: `{"p":1,"mtu":185,"fmax":63,"seg":0}` |

INFO fields:

| Key | Meaning |
|---|---|
| `p` | BLE link-layer version, **1**. |
| `mtu` | ATT MTU currently negotiated (23 until the exchange completes). |
| `fmax` | Largest frame the device sends or accepts without segmentation: `min(63, mtu − 3)`. |
| `seg` | `0` = one frame per ATT value (§4.3). `1` = segmented mode (§4.5, fallback only). |

The INFO value is rebuilt **in BT context** in the MTU-exchange event and on connect, and is never written from the main loop (F-7).

### 4.2 Advertising and discovery

- Connectable undirected advertising. Interval 100–150 ms while unconnected (library default if close), and stopped while connected (one connection, F-11).
- Payload, which must fit 31 bytes: Flags (3) + 128-bit service UUID (18) + Complete Local Name `TG-XXXX` (2 + 7 = 9) = **30 bytes**.
- `XXXX` is the last two bytes of the BD_ADDR in upper-case hex (e.g. `TG-1A2B`). The OLED and the `ble` CLI command show the same string, so a user can tell several stands apart.
- Web host filter: `requestDevice({ filters: [{ services: [SERVICE] }, { namePrefix: 'TG-' }], optionalServices: [SERVICE] })`. The `namePrefix` alternative covers browser apps that filter only on scan-response data.
- Security: `BLESecurityNone` (v1).

### 4.3 Frames on BLE

- The **same** frame as HID_CONTROL_PLANE.md §3.2 (the 63-byte WebHID buffer without a report ID): `flags, msgSeq, fragIndex, fragCount, payloadLen, chunk[payloadLen ≤ 58]`. All §3.3/§3.4 rules apply unchanged, including the 58-byte chunk rule, the 2048-byte message cap, at most 35 request fragments, the 500 ms reassembly timeout and one request at a time.
- **No padding on BLE**: the ATT value length is exactly `5 + payloadLen` (5..63). The device also accepts padded 63-byte writes, and ignores bytes after `5 + payloadLen`.
- **One frame per ATT value.** A value never carries two frames or part of one (unless `seg:1`, §4.5).
- A host write longer than 63 bytes is dropped as invalid.
- The host sends a request's frames **sequentially**, awaiting each GATT write before the next. Default `writeValueWithResponse`; `writeValueWithoutResponse` is optional and chosen in HW-07.
- The host must `startNotifications()` on TX **before** its first RX write.
- `msgSeq` counters are per link: the HID and BLE sessions number their messages independently.

### 4.4 Connection behaviour

| Event | Device behaviour |
|---|---|
| Connect | Store `con_handle`; rebuild INFO; ~1 s after connect, request connection parameters **interval 15–30 ms** (`min 12`, `max 24` in 1.25 ms units), latency 0, supervision timeout 4 s (Apple accessory guidelines). |
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
| `hello.link` (new object) | `{ "kind": "usbHid" \| "ble", "mtu"?: number }`. Describes **the link this request arrived on**; `mtu` is present only for BLE. |
| `hello.control` (new object) | `{ "owner": "usbHid" \| "ble" \| null, "self": boolean, "leaseMs": 10000 }`. Who holds control right now (§4.7). |
| `hello.caps.ble` | `true` when the firmware is built with BLE (`TG_BLE=1`), absent otherwise. |
| `hello.caps.cli` | `true` when the serial CLI is built (`TG_SERIAL_CLI=1`), absent otherwise (§5.7). |
| `hello.board` | `"Pico2W"` on W builds (`TG_BOARD_NAME`). |
| Op `claim` | Request `{ "takeover"?: boolean }`; result `{ "owner": "<kind>", "previous": "<kind>" \| null }`. §4.7. |
| Op `getLink` | Result `{ "usbHid": { "mounted": bool }, "ble"?: { "enabled", "name", "address", "advertising", "connected", "mtu", "connIntervalUs", "rxFrames", "txFrames", "rxDropped", "rxInvalid", "txRetries", "disconnects", "lastDisconnectReason" }, "control": <as hello.control> }`. Replaces the planned `ble` CLI command for non-CLI builds. |
| Op `getDiag` | Result: the `diagnostics_t` counters (`dmaOverruns`, `pcmRingUnderruns`, `pcmRingOverruns`, `pcm1808DmaBlockOverruns`, …, camelCase of the struct fields). Replaces CLI `diag` for HW tests. |
| Error `linkBusy` | A request from a non-owner link while another link holds control. `message`: `"control held by usbHid"` / `"… by ble"`. |
| Notify `controlReleased` | `{ "v":1, "notify":true, "op":"controlReleased", "data": { "to": "<kind>" } }` sent to the **previous** owner when another link takes over (if it's still connected). |
| TS types | `HidHelloLink`, `HidHelloControl`, `HidLinkStatus`, `HidDiag`, the `claim` / `getLink` / `getDiag` request/result types, `'linkBusy'` in `HidErrorCode`, `'controlReleased'` in the notify union, `HidHelloCaps.ble?` / `.cli?`, in `docs/hid/timegrapher_hid.d.ts` (synced to the web app via `npm run sync-types`). |

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
- Hosts that stay connected without sending requests (the web app on a passive view) must keep the lease alive: the web transports send `ping` after 3 s without any request (§6.2). WebHID gives the device no close signal, so without the lease a closed desktop tab would block BLE forever.
- RAM: one `Session` (reassembler + fragmenter, about 4.1 KB) per link, and **one** `ControlHandler`. The shared scratch buffers (`g_out`, `g_patch_obj`, …) stay single file-scope instances.

---

## 5. Firmware design detail

### 5.1 New and changed files

| File | Status | Owner |
|---|---|---|
| `include/control/frame.h`, `src/control/frame.cpp` | new | A1 |
| `include/control/hid_report.h` (header-only report-ID normaliser) | new | A1 |
| `include/control/link.h` (abstract `Link`) | new | A2 |
| `include/control/session.h`, `src/control/session.cpp` | new | A2 |
| `include/control/handler.h` (abstract `Handler`, `Reply`) | new | A2 |
| `src/control/control_handler.cpp` + `include/control/control_handler.h` | new (code moved from `hid_control.cpp`) | A2 |
| `include/control/arbiter.h`, `src/control/arbiter.cpp` (pure; §4.7) + gate responder | new | A2 |
| `getLink`, `getDiag` ops in `ControlHandler`; `hello.control`, `caps.cli` | new | A2 (`getLink` BLE fields filled by A3) |
| `src/display_ui.cpp`, `include/ui_snapshot.h`, `include/ui_format.h` (header layout helper), `docs/DISPLAY.md`, `tests/test_ui_format.cpp` | Bluetooth badge (§5.6) | A7 |
| `src/serial_cli.cpp`, `include/serial_cli.h`, `src/diagnostics.cpp`, `platformio.ini` (`[cli]`, `build_src_filter`), `scripts/calibrate_timebase.py` (legacy note) | CLI retirement (§5.7); `usb_audio.cpp` is **not** touched (CDC stays) | A8 |
| `src/hid_control.cpp`, `include/hid_control.h` | reduced to the HID adapter; public C API unchanged | A2 |
| `include/control/frame_ring.h` (header-only SPSC ring) | new | A3 |
| `include/control/ble_pacer.h` (header-only notify pacer, injected BTstack calls) | new | A3 |
| `include/ble_control.h`, `src/ble_control.cpp` (`#if TG_BLE`) | new | A3 |
| `src/TimegrapherStand_main.cpp` | `ble_control_begin()` after `usb_audio_init()`, `ble_control_task()` after `hid_control_task()` | A3 |
| `src/cli/cli_commands.cpp` | new `ble` command (CLI builds only; `getLink` is the primary interface) | A3 |
| `variants/rpipico2w_tg/` (project-local copy of the framework variant, `initVariant()` without radio start), `include/ble_core.h`, `src/ble_core.cpp` (radio bring-up on the selected core, LED proxy, core-0 hold), `setup1()`/`loop1()` changes in `TimegrapherStand_main.cpp`, `[ble_core1]` in `platformio.ini` | radio core selection (§5.8) | A9 |
| `include/board_config.h` | `BOARD_PICO2W`: `STATUS_LED_GPIO -1`, `STATUS_LED_CYW43 1`, `TG_BOARD_NAME "Pico2W"`, `TG_BLE` default 0 | A4 |
| LED helper (`led_set` in `TimegrapherStand_main.cpp`) | CYW43 LED branch | A4 |
| `platformio.ini` | `[ble]`, `[board_pico2w]` and `[cli]` sections, envs `pico2w`, `pico2w_synthetic`, `pico2w_pcm1808`, `pico2w_synthetic_noble`, `pico2_pcm1808_cli`, `pico2w_pcm1808_cli` | A4 (`[cli]` contents from A8) |
| `.github/workflows/ci.yml` | firmware matrix adds `pico2w_pcm1808`, `pico2w_synthetic`, `pico2_pcm1808_cli` (keeps the CLI compiling) | A4 |
| `tests/run_all.sh` | entries for the new host tests | A1, A2, A3 |
| `tests/test_control_frame.cpp`, `tests/test_control_session.cpp`, `tests/test_frame_ring.cpp`, `tests/test_ble_pacer.cpp` | new | A1, A2, A3 |
| `src/hid_json.cpp` / `include/hid_json.h` | `hello` encoder: `link` object, `caps.ble` | A2 (encoder takes a `link_kind` + `mtu` argument) |
| `docs/BLE_CONTROL_PLANE.md` | new | A5 |
| `docs/HID_CONTROL_PLANE.md`, `docs/AS_BUILT_ARCHITECTURE.md`, `docs/build_and_flash.md`, `docs/hid/timegrapher_hid.d.ts`, `docs/hid/webhid_example.md` | updated | A5 |
| `docs/hid/example.html` | "Connect (BLE)" option + Bluefy/Web Bluetooth feature probe | A6 |

Style for all firmware code: C++17, namespace `control` for the new modules, `noexcept` where nothing throws (no exceptions on target), no heap allocation after `setup()`, **Javadoc-style `/** … */` on every type, function and non-obvious member** (`@param`, `@return`, `{@code}`), clang-format clean (repo `.clang-format`), cppcheck clean (`--enable=warning,performance,portability`; inline `// cppcheck-suppress <id>` with a reason only where justified). Headers that C code includes keep `extern "C"`.

### 5.2 Framing module (`include/control/frame.h`) — A1

```cpp
namespace control {

constexpr std::uint8_t  kFrameHeaderBytes   = 5;
constexpr std::uint8_t  kFrameMaxChunk      = 58;
constexpr std::uint8_t  kFrameMaxBytes      = kFrameHeaderBytes + kFrameMaxChunk; // 63
constexpr std::uint16_t kMaxJson            = 2048;
constexpr std::uint8_t  kMaxRequestFragments = 35;   // 35 × 58 = 2030 bytes
constexpr std::uint32_t kReassemblyTimeoutMs = 500;
constexpr std::uint8_t  kFlagMore        = 0x01;
constexpr std::uint8_t  kFlagNotifyHint  = 0x02;

/** Outcome of FrameReassembler::push (for tests and diagnostics counters). */
enum class RxResult : std::uint8_t {
  kAccepted,       ///< Stored; message not complete yet.
  kComplete,       ///< Stored; message complete -> ready().
  kDuplicate,      ///< fragIndex already stored for this message; ignored.
  kBusy,           ///< A complete message awaits release(); frame ignored, state untouched.
  kShort,          ///< len < header or < header + payloadLen; ignored, state untouched.
  kInvalid,        ///< Header rule broken; reassembly state cleared.
  kCountMismatch,  ///< Same msgSeq, different fragCount; state cleared, frame dropped.
};

class FrameReassembler {
 public:
  RxResult push(const std::uint8_t *frame, std::size_t len, std::uint32_t now_ms) noexcept;
  void poll_timeout(std::uint32_t now_ms) noexcept;   // drops a stale partial message
  bool ready() const noexcept;
  const char *message() const noexcept;               // NUL-terminated, valid while ready()
  std::uint16_t length() const noexcept;
  void release() noexcept;                            // after dispatch; forgets the message
  void reset() noexcept;
 private: /* buffer[kMaxJson + 1], seq, fragCount, got, seen bitmask, deadline, ready */
};

class FrameFragmenter {
 public:
  bool begin(const char *json, std::size_t len, bool notify) noexcept; // false if active()
  bool active() const noexcept;
  /** Writes the current fragment into out[kFrameMaxBytes]; does NOT advance. */
  std::size_t peek(std::uint8_t *out, bool pad) const noexcept;        // 63 if pad, else 5 + chunk
  void commit() noexcept;                                              // advance after a successful send
  void abort() noexcept;
  std::uint8_t seq() const noexcept;
 private: /* buffer[kMaxJson + 1], len, off, seq, fragIndex, fragCount, notify, active */
};

} // namespace control
```

Behaviour must equal the current `on_host_report()` / `tx_begin()` / `tx_pump()` **exactly** (the FR-RX tests encode today's behaviour), except for one fix: `peek()`/`commit()` closes F-15. Details to keep:

- `len < 5` → `kShort` with no state change. `fragCount == 0`, `fragIndex ≥ fragCount`, `payloadLen > 58` or `fragCount × 58 > 2048` → `kInvalid`, and the state is cleared. `len < 5 + payloadLen` → `kShort` with no state change.
- A deadline that has passed (`(int32_t)(now − deadline) > 0`, deadline = first fragment + 500 ms) resets before the frame is processed, so the frame starts a new message.
- `begin()` clamps `len` to `kMaxJson`. An empty message is one fragment with `payloadLen 0`. `seq` is pre-incremented, so the first message is `msgSeq 1`.
- `hid_report.h`: `normalize_out_report(report_id, buf, len) → {ptr, len, ok}`, the "some stacks leave the report ID as the first byte" rule moved from `on_host_report()`.

### 5.3 Session, Link, Handler — A2

```cpp
namespace control {

/** One transport endpoint. All methods are main-loop only. */
class Link {
 public:
  virtual ~Link() = default;
  virtual const char *kind() const noexcept = 0;        ///< "usbHid" | "ble"
  virtual bool connected() const noexcept = 0;          ///< host present and listening
  virtual bool can_send() const noexcept = 0;           ///< room for one frame now
  virtual bool send_frame(const std::uint8_t *f, std::size_t n) noexcept = 0; ///< true = accepted
  virtual bool pad_frames() const noexcept = 0;         ///< HID true, BLE false
  virtual std::uint16_t mtu() const noexcept { return 0; } ///< 0 = not applicable
};

/** Where a handler puts its single outgoing message. */
class Reply {
 public:
  virtual ~Reply() = default;
  virtual bool idle() const noexcept = 0;
  virtual void send(const char *json, std::size_t n, bool notify) noexcept = 0; ///< only when idle()
  virtual const Link &link() const noexcept = 0;        ///< for hello.link
};

/** Protocol brain. Sessions reach it through the Arbiter. */
class Handler {
 public:
  virtual ~Handler() = default;
  virtual void on_request(const char *json, Reply &out) noexcept = 0; ///< must send exactly one reply
  virtual void poll(Reply &out, std::uint32_t now_ms) noexcept = 0;   ///< may send one notify if out.idle()
  virtual void reset() noexcept = 0;                                  ///< owner lost: clear subscriptions
};

/** Link kinds known to the arbiter. */
enum class LinkKind : std::uint8_t { kNone, kUsbHid, kBle };

/**
 * Ownership state machine (§4.7). Pure: no Arduino, no control_api, time
 * passed in. The firmware's Arbiter implements Handler for each Session and
 * forwards to the one ControlHandler or answers itself.
 */
class Arbiter {
 public:
  enum class Decision : std::uint8_t { kDispatch, kAnswerLocally, kBusy };
  struct Outcome {
    Decision decision;
    bool reset_handler;       ///< ownership moved: call ControlHandler::reset()
    LinkKind notify_released; ///< kNone, or the previous owner to send controlReleased to
  };
  /** Classifies one complete request (op name already parsed) from `from`. */
  Outcome on_request(LinkKind from, const char *op, bool takeover, std::uint32_t now_ms) noexcept;
  /** Owner link down → none + reset. Returns true if the handler must reset. */
  bool on_link_down(LinkKind kind) noexcept;
  LinkKind owner() const noexcept;
  bool lease_valid(std::uint32_t now_ms) const noexcept;
  static constexpr std::uint32_t kLeaseMs = 10000;
};

class Session final : private Reply {
 public:
  Session(Link &link, Handler &handler) noexcept;  // handler = this link's arbiter port
  void on_frame(const std::uint8_t *f, std::size_t n, std::uint32_t now_ms) noexcept;
  void task(std::uint32_t now_ms) noexcept;   ///< timeout, dispatch, pump, poll (current hid_control_task order)
  void on_link_down() noexcept;
  // Reply impl: idle(), send(), link()
 private:
  Link &link_; Handler &handler_; FrameReassembler rx_; FrameFragmenter tx_;
  bool link_was_connected_ = false;
};

} // namespace control
```

`Session::task()` keeps today's order exactly:

1. `rx_.poll_timeout(now)`.
2. If `rx_.ready()` and the fragmenter is idle, call `handler_.on_request(rx_.message(), *this)`, then `rx_.release()`.
3. Pump.
4. If the fragmenter is idle and `link_.connected()`, call `handler_.poll(*this, now)` and pump again.

The pump sends while `tx_.active() && link_.can_send()`: `peek` → `send_frame` → `commit` on success, stopping on failure. HID sends at most one frame per task call, as today, so USB timing is unchanged. BLE may send up to the free ring slots. The per-link limit is a constructor argument `max_frames_per_task` (HID 1, BLE 4).

`ControlHandler` is today's `handle_request` / `subscribe_poll` / `push_notify*` / encoders behind the `Handler` interface. There is **one instance**, so the `g_sub_*`, `g_last_*`, `g_pulse_*`, `g_beat_next_seq`, `g_last_event_peak` and `g_last_error_*` state may stay file-scope in `control_handler.cpp`. `reset()` sets the subscription to all-off and re-seeds the baselines (the same as a `subscribe` with everything off). The large scratch buffers stay file-scope statics, documented as "main loop only, one message at a time". `result_hello` uses `out.link()` for `hello.link` and the arbiter for `hello.control`.

**Arbiter port** (`src/control/arbiter.cpp` plus a small firmware glue file, `src/control/arbiter_port.cpp`): each Session's `Handler&` is an `ArbiterPort(kind)`.
- `on_request` parses `op` (and `takeover` for `claim`) with the existing `json_get_string` / `json_get_bool` helpers, which move into a small shared `control/json_scan.h`, and asks `Arbiter::on_request`.
- On `kDispatch` it forwards to the `ControlHandler`.
- On `kAnswerLocally` it encodes `hello` (identity + link + control; it may call the same `hid_json` hello encoder the handler uses), `ping` or the `claim` result.
- On `kBusy` it encodes the `linkBusy` error with the request `id`.
- When `reset_handler` is set it calls `ControlHandler::reset()`. When `notify_released` names a connected link, it queues `controlReleased` on that session: one pending-notify flag per session, sent when that session's TX goes idle.
- `poll()` forwards to the `ControlHandler` only if this port's kind is the owner.

`hid_control.cpp` keeps the descriptor (with its `clang-format off/on` and cppcheck suppression), `g_hid`, `UsbHidLink`, a static `Session g_hid_session` bound to the HID arbiter port, `on_host_report()` → `normalize_out_report` → `g_hid_session.on_frame(…, millis())`, and `hid_control_task()` → `g_hid_session.task(millis())`. `UsbHidLink::connected()` = `TinyUSBDevice.mounted()`. `can_send()` = `g_hid.ready()`. `pad_frames()` = true. A transition from mounted to unmounted calls `on_link_down()`. This also fixes a latent issue: today subscriptions survive a USB re-plug.

**Behavioural invariant for A2:** apart from F-15 and the unmount reset, nothing observable over USB HID changes. Every existing host test, the web contract suite against real hardware (manual) and `docs/hid/example.html` keep working.

### 5.4 BLE link — A3

**Header-only, host-testable pieces:**

- `control::FrameRing<N>`: SPSC queue of `N` slots of `{uint8_t len; uint8_t data[64];}` with free-running `std::atomic<uint16_t>` head and tail (acquire/release), `push(const uint8_t*, size_t) → bool` (false when full or `len > 64`, counting `overflows`), `pop(uint8_t*, size_t*) → bool`, `size()`, `clear()` (consumer side only), and `static_assert(std::atomic<uint16_t>::is_always_lock_free)`. RX ring `N = 40` (a full 35-fragment request plus margin). TX ring `N = 16`.
- `control::BlePacer`: the can-send state machine, with BTstack calls injected as function pointers (`request_can_send(ctx)` and `notify(con, data, len) → int`) so it can be host-tested. `kick()` runs **in BT context only**, from the TX worker (§5.8): the main loop pushes a frame and calls `async_context_set_work_pending(ctx, &g_tx_worker)`, and the worker calls `kick()`. `kick()` registers exactly once while nothing is pending. `on_can_send()` runs in BT context: it peeks one frame, calls `notify`, pops on success (status 0), keeps the frame on error, and re-registers if frames remain or the send failed. `on_disconnect()` flushes and clears pending.

**`src/ble_control.cpp` (`#if TG_BLE`):**

- `class TgTxCharacteristic : public BLECharacteristic` overrides `setConHandle()` / `disconnected()` to publish `con_handle` (atomic) and to read `_configData & 0x0001` for the CCCD notify bit.
- `class TgRxCharacteristic` (or `BLECharacteristicCallbacks`): `onWrite` in BT context pushes `valueData()`/`valueLen()` into the RX ring. A length > 63 counts as `rx_invalid`.
- An INFO characteristic is rebuilt in BT context (F-7) on connect and on `ATT_EVENT_MTU_EXCHANGE_COMPLETE`. Register an extra HCI/ATT event handler (`hci_add_event_handler` / `att_server_register_packet_handler`, whichever composes with the library; check that the library's handler still runs) to see MTU and connection-update events.
- `BleLink : control::Link`: `connected()` = con_handle ≠ 0 && CCCD on. `can_send()` = connected && TX ring not full. `send_frame()` = push + `async_context_set_work_pending()` (no lock, F-22). `pad_frames()` = false. `mtu()` = the stored MTU.
- `ble_control_begin()`: build the name `TG-XXXX` from `BLE.address()`, set up the service and characteristics, `BLE.begin(name)`, `BLE.server()->addService()`, `BLE.startAdvertising()` (**once**, F-27), and register the TX worker with `async_context_add_when_pending_worker()`. It runs on the radio core (§5.8): from `setup()` after `usb_audio_init()` with `TG_BLE_CORE=0`, from `setup1()` with `TG_BLE_CORE=1`. The CYW43 bring-up time is measured in HW-01 and must not delay USB enumeration beyond the 3 s in HW-01.
- `ble_control_task()` (main loop): drain the RX ring into `g_ble_session.on_frame()`; handle link-down/up flags set from BT context (→ `on_link_down()`); then `g_ble_session.task(millis())`. It makes **no BTstack calls**. Requests for BT context (resume advertising with `gap_advertisements_enable(1)`, the connection-parameter request 1 s after connect, `ble off`/`on`) are atomic flags that the TX worker handles.
- `ble_control_status(ble_status_t *)` for the CLI and the display: enabled, name, address, advertising, connected, cccd, mtu, conn interval (µs), rx_frames, tx_frames, rx_dropped, rx_invalid, tx_retries, disconnects, last disconnect reason.
- CLI `ble` (in `cli_commands.cpp`, same style as `diag`): `ble` prints the status. `ble off` / `ble on` are runtime-only (stop advertising and disconnect, or start again; for A/B audio tests, not persisted). `ble reset` clears the counters. `ble help`. Without `TG_BLE`, `ble` prints `CLI: ble not built (TG_BLE=0)`.

**Milestones:** A3 delivers in two steps so hardware can be tested early.

- **M1 "echo"** (needed for gate G1): the GATT service, rings, pacer, INFO, CLI and advertising, with `ble_control_task()` in **echo mode** (each received frame is sent straight back). Selected at compile time with `-DTG_BLE_ECHO=1`; there is no separate env, and A3 documents the one-line override in `docs/build_and_flash.md`.
- **M2 "session"**: wire `BleLink` to a `Session` bound to the BLE arbiter port. Fill the `ble` object of `getLink` from `ble_control_status()`, and publish `ble_enabled` / `ble_connected` into the UI snapshot for the badge (§5.6). Needs A2.

### 5.5 Build targets — A4

```ini
; BLE control plane (Pico 2 W only). See docs/BLE_CONTROL_PLANE.md.
[ble]
build_flags =
	-DPIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH
	-DTG_BLE=1

; Raspberry Pi Pico 2 W (RP2350A + CYW43439). Radio owns GPIO23/24/25/29:
; the status LED is the CYW43 LED, never GPIO25 (board_config.h).
[board_pico2w]
extends = board_pico2
board = rpipico2w
build_flags =
	${board_pico2.build_flags}
	-DBOARD_PICO2W=1

[env:pico2w]            ; ADC, like [env:pico2]
[env:pico2w_synthetic]  ; synthetic source (audio-integrity tests)
[env:pico2w_pcm1808]    ; PCM1808, same pins as pico2_pcm1808
[env:pico2w_synthetic_noble] ; W board, BLE NOT built (isolation for HW-01 / HW-02; not in CI)
```

Each BLE env's `build_flags` = `${board_pico2w.build_flags}` + `${ble.build_flags}` + its audio-source flags (copied from the matching pico2 env). `board_config.h` handles `BOARD_PICO2W` **before** the `BOARD_PICO2` branch, because W builds define both. `TG_BLE` defaults to 0 under `#ifndef`.

CLI opt-in (contents owned by A8, §5.7):

```ini
; Legacy serial (CDC) CLI. Omitted by default: USB HID and BLE are the control
; planes. Add ${cli.build_flags} and ${cli.build_src_filter} to an env (or set
; PLATFORMIO_BUILD_FLAGS=-DTG_SERIAL_CLI=1 together with the src filter) to bring it back.
[cli]
build_flags = -DTG_SERIAL_CLI=1
build_src_filter = +<*>

[base]
...
; CLI sources are filtered out unless an env opts in via [cli].
build_src_filter = +<*> -<cli/> -<serial_cli.cpp>

[env:pico2_pcm1808_cli]       ; CI keeps the CLI compiling
[env:pico2w_pcm1808_cli]      ; W + BLE + CLI, hardware day (not in CI)
[env:pico2w_synthetic_cli]    ; W + BLE + CLI + synthetic source: HW-02/HW-06 A/B and the G1 echo build (not in CI)
```

`pico2w_synthetic_noble` (§5.5) **includes** `[cli]`, because it exists only for hardware A/B diagnostics. So the G0 A/B is `pico2w_synthetic_noble` vs `pico2w_synthetic_cli`: identical except for BLE.

### 5.6 OLED Bluetooth badge — A7

**Look:** same style as the GPS pill (`draw_gps`), at the same height (y 2, 11 px tall).
- An 11 × 11 rounded rectangle (radius 3) with a 5 × 9 Bluetooth rune centred in it (origin x0+3, y0+1).
- **Hollow** (`drawRoundRect`, white rune) while BLE is enabled but **not connected** (advertising).
- **Inverted** (`fillRoundRect` white, rune drawn black) while a central is **connected** with notifications on (`BleLink::connected()`).
- **Not drawn** on builds without `TG_BLE`, or while BLE is turned off (`ble off`, CLI builds).

Starting bitmap (MSB = left pixel, 5 bits used), to refine on hardware (HW-17):

```text
row 0  ..X..  0x20      row 5  .XXX.  0x70
row 1  ..XX.  0x30      row 6  X.X.X  0xA8
row 2  X.X.X  0xA8      row 7  ..XX.  0x30
row 3  .XXX.  0x70      row 8  ..X..  0x20
row 4  ..X..  0x20
```

**Placement:** right-to-left: GPS pill (when `pps_enabled`) at x 105, then the BT badge 2 px left of it (x 92; x 117 without GPS), then the mode tag right-aligned 2 px left of the leftmost badge. Both the mechanical and the quartz header use the same rule.

**Make the layout a pure, tested function.** A new `ui_header_layout(bool pps, bool ble, bool standalone)` in `include/ui_format.h` returns the x positions (GPS, BT, tag start/end) and `max_left_px`, the right edge the left-hand text must stay below. `display_ui.cpp` uses it for both headers. Host tests (UIH-*) prove that no combination overlaps.

**Making room (decision D-UI, §12; default below):** with GPS + BT + `USB`, `max_left_px` is 70 (11 characters at size 1).
- **Mechanical header:** shorten the three status strings longer than 11 characters, in `ui_format.h` (and therefore also on the quartz status line, which shares the wording):

  | Today | New |
  |---|---|
  | `no host stream` | `no stream` |
  | `clock check...` | `clock check` |
  | `listening...` | `listening` |

  `docs/DISPLAY.md` and `tests/test_ui_format.cpp` are updated to match.
- **Quartz header:** `QUARTZ` (36 px) + 6 px + `UNCAL` badge (36 px) would end at x 78. The rule is per tag (as built by A7): when that tag's full `UNCAL` badge would end past `max_left_px`, the badge is drawn **directly after the tag with a 3 px gap** and reads `UNC` (24 px including padding, ending at x ≤ 63 for `QUARTZ`); otherwise it's unchanged. With BLE + PPS + `USB`, `QUARTZ` and `MOTOR` get `UNC`, `LCD` and `FORK` keep `UNCAL`; without BLE `UNCAL` is always kept.

**Data path:** `ui_snapshot_t` gets `bool ble_built; bool ble_enabled; bool ble_connected;`, published by core 0 in `publish_ui_snapshot()` from `ble_control_status()`. Core 1 never calls BLE code.

### 5.7 Retiring the serial CLI — A8

**Flag:** `TG_SERIAL_CLI`. Defined (1) only by `[cli]`; every shipped env omits it. `#ifndef TG_SERIAL_CLI` → `0` in `app_config.h`.

**When omitted, remove as much as possible:**

| Item | Mechanism |
|---|---|
| `src/cli/cli_commands.cpp`, `src/cli/cli_core.cpp`, `src/serial_cli.cpp` | `build_src_filter` in `[base]` excludes them (they aren't compiled at all) |
| `serial_cli_init()` / `serial_cli_tick()` calls | `include/serial_cli.h` provides `static inline` no-op versions when `!TG_SERIAL_CLI`, so `main` needs no `#if` |
| `diagnostics.cpp` periodic `Serial` print and `diagnostics_set_print_enabled` | compiled out (`#if TG_SERIAL_CLI`); the counters stay (`getDiag`) |
| Any other `Serial.` use outside the CLI (audit: `grep -rn "Serial\." src include`) | removed or guarded; the review fails if an unguarded use remains (CLI-04) |
| **CDC interface on the USB bus** | **Kept** (revision 3). It carries no CLI, but the library's 1200-baud touch still reboots into BOOTSEL for `pio run -t upload` (F-18, F-20). Input on the port is ignored. Nothing is written to it except the library's one UUID line at boot (F-12). |
| `include/cli/device_format.h`, `quartz_format.h` | header-only; they cost nothing unless included. Their host tests keep running (they test pure formatting) |

**HID parity:** A8 audits every CLI command (`help version status time timebase device run display diag dsp quartz mode gate fork ts ksigma restart pulses inhibition jumpers`) against the HID ops. It writes the table into `docs/HID_CONTROL_PLANE.md` (new appendix "CLI → HID") with an equivalent op, "new op" or "dropped (reason)" for each. The gaps already known are `getDiag` and `getLink` (§4.6), which A2 implements. Upload reset needs no op (CDC stays). Any further gap A8 finds is reported to the manager; it isn't silently added.

**Consequences, documented in `docs/build_and_flash.md`:**
- `pio run -t upload` works as before on every build: the serial port still appears and still accepts the 1200-baud reset. A terminal opened on it shows nothing (no CLI).
- `scripts/calibrate_timebase.py` needs a `*_cli` build. The web app's PC-assisted calibration is the supported path.
- Hardware-day diagnostics use `getDiag` / `getTiming` / `getLink` over HID or BLE, or the `*_cli` envs.

### 5.8 CPU allocation and radio core — A9

**Review question (revision 3):** with the radio added, does "core 0 does everything time-critical, core 1 runs the display" still fit?

**Today's split** (`TimegrapherStand_main.cpp`, `display_ui.cpp`):

| Core | Work | Deadline |
|---|---|---|
| 0, thread (`loop()`) | `TinyUSBDevice.task()`, capture block intake, DSP graph + quartz engine, HID control (JSON replies up to 2 KB), timebase, diagnostics, UI snapshot, LED | Capture has 2 DMA buffers of 5 ms, so each block must be taken within ~5 ms. The UAC2 feed runs every 1 ms (the 20 ms ring prefill absorbs DSP jitter, not a stalled loop). `solve_incremental()` takes ≤ ~2 ms |
| 0, IRQ | USB, capture DMA | hardware |
| 1, thread (`loop1()`) | OLED: render, ~55 ms blocking I²C per frame, sleep to the 500 ms period | none (cosmetic) |

That split is still right for everything that exists today. USB must stay on core 0: TinyUSB is brought up there before `setup()`. The DSP and quartz engine share state with `control_api`, which HID and BLE requests mutate from core 0. Moving them to core 1 would put them next to the display's 55 ms blocking I²C and add cross-core locking to every parameter change: a large refactor with no measured need. **Unchanged: USB, capture, DSP/quartz, framing, sessions, arbiter, `control_api` and JSON stay on core 0. The display stays on core 1.**

**What BLE adds:**

- **Control work:** framing, session, arbiter and JSON. It costs the same as HID and belongs with `control_api`, so it runs on core 0 in either placement. Exclusive control (§4.7) means only one link generates heavy traffic.
- **Radio work:** CYW43 driver plus BTstack, as the lowest-priority IRQ on whichever core initialised the radio (F-17, F-22):
  - USB and DMA interrupts still pre-empt it, but it pre-empts the thread code on its core.
  - It includes busy-waited SPI transfers (F-23).
  - It's bursty: connect, MTU exchange, GATT discovery (Bluefy reads every characteristic), and notify bursts at the subscription rates.
  - It's the only non-real-time, externally triggered workload in the system. Core 1 is idle 85–90 % of the time (F-26).

**Options for the radio:**

| | **A: core 0** (`TG_BLE_CORE=0`, override) | **B: core 1** (`TG_BLE_CORE=1`, default, next to the display) |
|---|---|---|
| What core 0 sees | Every BT IRQ burst pre-empts `loop()` (USB task, DSP) | Only lock-free ring pushes/pops, atomics and `set_work_pending()` |
| Pros | No extra code; the framework's tested path; flash commits already stop it (`noInterrupts()`) | Core 0 timing stays as today's measured baseline; BT bursts pre-empt only the display, which has no deadline |
| Cons | Radio latency lands directly on the 5 ms capture and 1 ms USB deadlines (R-1) | A project-local variant (F-24); an extra hardware alarm (F-22); core 1's 8 KB stack shared with BT IRQs (F-26); core 1 must run on no-display W builds; boot-order care (rule 6) |
| Flash commit (`EEPROM.commit()`) | Radio stalls with core 0's interrupts off | Radio stalls while core 1 is parked by `idleOtherCore()` |

A flash commit stalls the radio for the same tens of ms in either placement. Commits never happen while streaming, and the link-layer supervision timeout (4 s) runs in the controller, so the link survives (HW-14).

**Decision (2026-11-23, user, after HW-19):** B is the default for every W env. HW-19 (30 s, BLE subscribed at caps): core 1 had 0 audio discontinuities and a longest loop pass of 6.5 ms; core 0 had 5 discontinuities and 8.7 ms. A 10 min soak on core 1 was clean, and `core1StackFree` stayed at 5.4 to 6.7 KB (CORE-05). `[ble]` passes `-DTG_BLE_CORE=1`, `[board_pico2w]` selects the project variant, and env `pico2w_synthetic_core1` is removed. A stays buildable as an override with the stock variant (`docs/build_and_flash.md`, "Radio core"); `tests/check_ble_core.sh --build` checks both placements.

**Recommendation (before HW-19):**
- Build both placements behind `TG_BLE_CORE`. A3-M1 brings the radio up on **A** first (gates G0/G1 need no new mechanism).
- A9 adds **B**. HW-19 compares them on hardware.
- **Decision rule:** choose B if it passes HW-02/HW-06/HW-13. Choose A if B has a blocker (R-14) and A meets HW-19's limits. If neither does, escalate via the R-1 ladder (a)–(c).
- B is preferred because it keeps the real-time core's timing independent of what a phone does over the air.

**Placement-independent rules (A3 follows them from M1; A9 relies on them):**

1. **No `BluetoothLock` and no BTstack calls in `loop()` code.** On the radio's own core, releasing the lock runs pending BT work inline (F-22); on the other core it blocks while BT work runs.
   - TX: `send_frame()` pushes to the TX ring, then calls `async_context_set_work_pending(ctx, &g_tx_worker)`.
   - Other requests (resume advertising, the connection-parameter request, `ble off`/`on`, counter reset) are atomic flags that the same worker reads.
2. **BT context touches only rings, atomics and the INFO buffer** (unchanged from §5.4). It never calls `Session`, `control_api`, `Serial` or TinyUSB.
3. **`ble_control_begin()` runs on the radio core.** Code after it must not assume which core that is (`get_core_num()` is only used in asserts).
4. **LED via BT context** (F-25). `led_set()` on W builds stores the wanted state in an atomic and calls `set_work_pending()` on an LED worker, which calls `cyw43_arch_gpio_put()`. Non-W boards keep `digitalWrite`.
5. **`startAdvertising()` once** (F-27). Resume uses `gap_advertisements_enable(1)` in the worker.
6. **Core-0 hold for B** (F-12, F-18):
   - The UUID line printed by `startAdvertising()` must not run on core 1 while core 0 runs TinyUSB (the CDC FIFO isn't multi-core safe).
   - With `TG_BLE_CORE=1`, `setup()` waits after `usb_audio_init()` on an atomic "radio ready" flag that core 1 sets after `ble_control_begin()`, with a 3 s cap.
   - The wait is a plain spin: no `delay()`, no `yield()`, no `TinyUSBDevice.task()`.
   - On timeout, core 0 continues and BLE reports `enabled=false`.

**B mechanism (A9):**

- **Variant:**
  - Copy `variants/rpipico2w/` to `variants/rpipico2w_tg/`, with its `init.cpp` reduced to an empty `initVariant()` when `TG_BLE_CORE=1`. With `TG_BLE_CORE=0` the variant isn't used.
  - Select it with `board_build.variants_dir = variants` and `board_build.variant = rpipico2w_tg` in `[board_pico2w]` (was `[ble_core1]`).
  - Keep a header comment naming the framework version it was copied from (5.5.1); CORE-04 checks that `pins_arduino.h` is unchanged.
- **`src/ble_core.cpp`:**
  - `ble_core_setup1()` calls `init_cyw43_wifi()` (F-24) and then `ble_control_begin()` on core 1, then sets "radio ready".
  - `setup1()` calls it before `display_ui_setup()`.
  - W builds without a display still define `setup1()`/`loop1()`: `loop1()` calls `display_ui_loop()` or idles with `__wfe()`, and `multicore_reset_core1()` is skipped when `TG_BLE_CORE=1`.
- **Stack:**
  - Paint core 1's 8 KB stack at boot and report the high-water mark in `getLink.ble.core1StackFree` (B only).
  - CORE-05 requires ≥ 2 KB free after HW-13. If it fails, raise the stack: `core1_separate_stack` is fixed at 8 KB in `main.cpp`, so a larger stack needs `multicore_launch_core1_with_stack` from a project-owned launch, which A9 reports to the manager before doing.
- **Build:**

  ```ini
  ; Radio on core 1 next to the display (§5.8), the default of every BLE env.
  [ble]
  build_flags = ... -DTG_BLE=1 -DTG_BLE_CORE=1

  [board_pico2w]
  board_build.variants_dir = variants
  board_build.variant = rpipico2w_tg
  ```

  Since 2026-11-23 there is no separate core-1 env. `TG_BLE_CORE` defaults to the value of `TG_BLE` under `#ifndef` (1 with BLE) and is only meaningful with `TG_BLE=1`. The variant reads the command-line flag, so a core-0 override passes `-DTG_BLE_CORE=0` explicitly, with the stock variant.
- **`hello`/`getLink`:** `getLink.ble.core` is `0` or `1`, so test records show the placement.

---

## 6. Web app design detail (`timegrapher-web`)

### 6.1 Files

| File | Status | Owner |
|---|---|---|
| `src/transport/FramedTransport.ts` (+ `.test.ts`) | new abstract base | W1 |
| `src/transport/WebHidTransport.ts` | now extends `FramedTransport`; public API and behaviour unchanged | W1 |
| `src/transport/Transport.ts` | `TransportKind` adds `'webble'`; `ClientErrorCode` adds `'linkUnsupported'` | W2 |
| `src/protocol/constants.ts` | `BLE_SERVICE_UUID`, `BLE_RX_UUID`, `BLE_TX_UUID`, `BLE_INFO_UUID`, `BLE_NAME_PREFIX = 'TG-'`, `BLE_MIN_MTU = 66` | W2 |
| `src/transport/WebBleTransport.ts` (+ `.test.ts`) | new | W2 |
| `src/transport/fakeBluetooth.ts` (test support, under `tests/support/` if that is the repo convention) | new | W2 |
| `tests/contract/bleSimHarness.ts`, `tests/contract/contract.ble.test.ts` | new | W2 |
| `src/stores/connection.ts` (+ tests) | factory `'webble'`, `webBleAvailable`, link-aware label | W3 |
| `src/stores/subscriptions.ts` (+ tests) | link profile: per-kind rate caps (§6.3) | W3 |
| `src/components/ConnectPrompt.vue`, `src/components/AppBarConnection.vue` (+ tests) | BLE button; iOS/Bluefy guidance | W3 |
| `src/utils/platform.ts` (+ test) | `isIos()`, `isLikelyBluefy()` (pure, injectable UA/navigator) | W3 |
| `package.json` | devDependency `@types/web-bluetooth` (pinned exact) | W2 |
| `src/protocol/timegrapher_hid.d.ts` | **only** via `npm run sync-types` after A5 | W4 |
| `README.md`, `docs/ARCHITECTURE.md` | BLE section, Bluefy instructions | W4 |

Style: strict TypeScript (no `any`, no non-null `!` unless justified, `exactOptionalPropertyTypes` as configured), **JSDoc on every export and non-trivial member**, Vue `<script setup lang="ts">`, the repo's ESLint and Prettier, Vitest with happy-dom. Run `npm run lint && npm run typecheck && npm test && npm run build` before submitting.

### 6.2 `WebBleTransport`

- `constructor(options: { bluetooth?: Bluetooth; device?: BluetoothDevice; defaultTimeoutMs?: number; writeMode?: 'withResponse' | 'withoutResponse'; mtuWaitMs?: number })`. The defaults are `navigator.bluetooth`, 4000 ms, `'withResponse'` and 300 ms × 3 retries.
- `open()`:
  1. `requestDevice` (§4.2 filters). A `NotFoundError` (user cancelled) becomes `DeviceError('notOpen', 'No device selected')`. Use this exact message: the connection store keys its quiet cancel on it.
  2. `gatt.connect()`, then `getPrimaryService(SERVICE)`, then the RX, TX and INFO characteristics.
  3. Read INFO and parse it. If `mtu < 66` and `seg !== 1`, re-read up to 3 times, 300 ms apart. If it's still too small, close and reject `DeviceError('linkUnsupported', 'BLE link MTU <n> is below 66; this browser/OS cannot run the control plane')`.
  4. `tx.startNotifications()`, add a `characteristicvaluechanged` listener, add a `gattserverdisconnected` listener.
- `sendFrame(frame)`: unpadded (`5 + payloadLen` bytes), awaited. Exactly one GATT operation is in flight at a time, across all frames and requests.
- **Keepalive (in `FramedTransport`, so WebHID gets it too):** after 3000 ms without any request, the transport sends `ping` (2 s timeout; a failed keepalive is logged and otherwise ignored). It keeps the 10 s ownership lease (§4.7) alive on passive views.
- **Ownership:** `linkBusy` errors propagate as `DeviceError('linkBusy', …)`. A `controlReleased` notify makes the transport tear down with reason `"Control taken over by <kind>"` (disconnect listeners fire, like an unplug).
- Received values go to the shared `FrameReassembler`. Values longer than 63 bytes are dropped.
- `close()`: idempotent. It stops notifications (ignoring errors), removes listeners and calls `gatt.disconnect()`. It does **not** fire disconnect listeners, matching `WebHidTransport`.
- An unexpected `gattserverdisconnected` rejects every pending request with `'disconnected'` and fires the disconnect listeners once.
- `productName` = `device.name`.

### 6.3 Link profile (rate caps over BLE)

These are provisional and finalised from HW-07 measurements. `subscriptions` clamps requested rates by link kind:

| Rate | `webhid` | `webble` (provisional) |
|---|---|---|
| `metricsHz` | 20 | 10 |
| `beatHz` | 5 | 2 |
| `timingHz` | 5 | 2 |
| `quartzHz` | 5 | 2 |

Keep the table in one exported constant (`LINK_RATE_CAPS`) so the manager can retune it after hardware day.

### 6.3a Exclusive control in the app (W3)

- The app already holds one transport at a time (`connection` store). Connecting over one kind first disconnects any other.
- After `hello`, if `hello.control.owner` is set and `self` is false, the store does **not** proceed. It shows a dialog: "This stand is being controlled over USB (or Bluetooth) by another app. Take over?", with **Take over** (sends `claim {takeover:true}`, then continues the normal connect) and **Cancel** (disconnects).
- Any later `linkBusy` (ownership changed under us) shows the same dialog.
- `controlReleased`: a warning snackbar "Another app took control over <kind>", and the store resets as for an unplug.
- Firmware without `hello.control` (old builds) connects as today.

### 6.4 iOS guidance (W3)

- On iOS/iPadOS **without** `navigator.bluetooth` (Safari, Chrome iOS), the connect prompt explains that iPhone/iPad browsers can't talk to the device and suggests opening this page in a Web Bluetooth browser such as **Bluefy**, with a "Copy link" button. Don't hard-code App Store links or URL schemes (they're unverified).
- With `navigator.bluetooth` but no `navigator.hid` (Bluefy, Android Chrome), show the BLE and Simulator buttons and hide the WebHID hint.
- Detection is in `src/utils/platform.ts`, with the user agent and navigator injected so it's unit-testable. iPadOS reports a desktop UA, so treat `MacIntel` with `maxTouchPoints > 1` as iPad.

---

## 7. Dependency graph and schedule

```text
Day 0 (2026-11-22, no hardware)                                   Day 1 (2026-11-23, hardware)
A4 build/board ─┬──────────────► A3-M1 echo ─┬───────────► G0 / G1 hardware gates ──┐
                │                             └─► A9 radio core (TG_BLE_CORE) ───────┤
A1 framing ─────┼─► A2 session + arbiter ─► A3-M2 BLE session ──────────────────────┼─► G2 integration + soak
A8 CLI retire ──┤        └──────────────► A7 OLED BT badge ─────────────────────────►│
A5 docs (spec)  │                         A6 example.html BLE (spec-driven) ───────►│
W1 framed base ─┴─► W2 WebBleTransport + contract ─► W3 store/UI/takeover ─► W4 ───┘
```

A8, A2, A3 and A9 all touch `TimegrapherStand_main.cpp` / `platformio.ini` lightly. The manager sequences them: **A8 after A4, before A2 starts on `hid_control.cpp`**. A8's file list doesn't include `hid_control.cpp`. **A9 goes after A3-M1** (it moves `ble_control_begin()`), and touches `setup1()`/`loop1()`, which A7 doesn't.

| Task | Model | Depends on | Can run in parallel with | Target |
|---|---|---|---|---|
| A4 build/board/CI | **haiku** | — | A1, A5, W1 | Day 0 |
| A1 framing extraction | **sonnet** | — | A4, A5, W1 | Day 0 |
| A5 docs + d.ts | **haiku** | — (spec frozen here) | everything | Day 0, final pass Day 1 |
| W1 FramedTransport | **sonnet** | — | firmware tasks | Day 0 |
| A8 CLI retirement + HID parity audit | **sonnet** | A4 | A1, A5, W1 | Day 0 |
| A2 session + arbiter refactor | **sonnet** | A1, A8 | A3-M1, W2 | Day 0 |
| A7 OLED Bluetooth badge | **sonnet** | A4 (snapshot fields may be stubbed before A3-M2) | A2, A3 | Day 0 (host tests), Day 1 (HW-17) |
| A3-M1 BLE echo | **sonnet** | A4 (envs), A1 (constants) | A2 | Day 0 (compile), Day 1 (G1) |
| W2 WebBleTransport | **sonnet** | W1; A5 for the `hello.link` types | A2, A3 | Day 0 |
| A6 example.html BLE | **sonnet** | §4 spec only | A2, A3 | Day 0 |
| A9 radio core selection (`TG_BLE_CORE`) | **sonnet** | A3-M1, A4 | A2, A7, W2 | Day 0 (compile + CORE host/static tests), Day 1 (HW-19) |
| A3-M2 BLE session | **sonnet** | A2, A3-M1, G1 pass | W3 | Day 1 |
| W3 store/UI/iOS guidance | **sonnet** | W2 | A3-M2 | Day 0–1 |
| W4 web docs + sync-types | **haiku** | A5, W3 | — | Day 1 |

Model rationale (per the standing workflow): **sonnet** for protocol and framing logic, concurrency (SPSC/BT context), firmware refactors, transports and non-trivial UI/state. **haiku** for build config, CI YAML, docs sync and mechanical type syncing. The manager (Opus) does every review and all verification.

---

## 8. Sub-agent task briefs

Each brief is the starting prompt for that agent. The manager adds the brief, then the standing rules below, then any review findings.

**Standing rules for every agent**

1. Read this plan (§2 facts, §4 spec, the task's section) and the test IDs you own in `BLE_TRANSPORT_TESTS.md` **before** coding.
2. **Never run `git commit`, `git push`, `git stash`, `git reset` or `git checkout -- <file>`.** Leave changes in the working tree. `git add -N` for new files is fine.
3. Touch only the files your brief lists. If you need another file, stop and say why in your report.
4. Every new test listed for you must exist, be wired into the runner, and pass. Report the exact commands you ran and their output tail.
5. Documentation comments on everything you add (Javadoc style in C++, JSDoc in TS). Explain *why*, not just *what*.
6. No new dependencies beyond those named in your brief.
7. End your report with: files changed, tests added (IDs → test names), commands run plus results, open questions, and anything you deliberately did not do.

### A1 — Framing extraction (sonnet)

**Context:** `src/hid_control.cpp` (`on_host_report`, `rx_reset`, `tx_begin`, `tx_pump`, constants at the top of the file), `docs/HID_CONTROL_PLANE.md` §3, plan §5.2, F-10 and F-15.
**Do:**
- Create `include/control/frame.h` + `src/control/frame.cpp` and `include/control/hid_report.h` as in §5.2.
- Write `tests/test_control_frame.cpp` covering **FR-RX-01..20, FR-TX-01..10, FR-RT-01..03, FR-HID-01..03**, and add `entry control_frame cxx tests/test_control_frame.cpp src/control/frame.cpp` to `tests/run_all.sh`.
- Do **not** change `hid_control.cpp` yet. A2 switches it over, so this task can't change device behaviour.

**Acceptance:** `tests/run_all.sh` all PASS (also `SANITIZE=1`); `pio run -e pico2_pcm1808` builds (the module compiles for target even though it isn't linked yet: add it to the source filter only if needed, otherwise leave it for A2).
**Files:** the 4 new files plus `tests/run_all.sh`.

### A2 — Session / Link / Handler / Arbiter refactor (sonnet)

**Context:** `src/hid_control.cpp` (the whole file), `include/hid_control.h`, `src/hid_json.cpp`/`include/hid_json.h` (hello encoder), `include/diagnostics.h`, plan §4.6, §4.7, §5.3 and A1's `frame.h`.
**Do:**
- Create `link.h`, `handler.h`, `session.h/.cpp`, `arbiter.h/.cpp`, `arbiter_port.cpp`, `json_scan.h` and `control_handler.h/.cpp`.
- Move the dispatch, encoders and subscription engine into the single `ControlHandler`. File-scope state may stay; add `reset()`.
- Reduce `hid_control.cpp` to the adapter (§5.3), keeping the public C API.
- Add `hello.link`, `hello.control`, `caps.ble` (only when `TG_BLE`) and `caps.cli` (only when `TG_SERIAL_CLI`).
- Add the ops `claim`, `getLink` (with `usbHid` filled; the `ble` object comes from a weak hook `ble_control_status()` that A3 implements; absent without `TG_BLE`) and `getDiag`, plus the `linkBusy` error and `controlReleased` notify (§4.6, §4.7).
- Write `tests/test_control_session.cpp` covering **SES-01..14** with a fake `Link` and fake `Handler` (no `control_api`), and `tests/test_control_arbiter.cpp` covering **ARB-01..16** (pure `Arbiter`). Wire both in `run_all.sh`.
- Extend `tests/test_hid_json.cpp` for **HJ-01..06** (hello link/control/caps, getLink, getDiag shapes).

**Constraints:**
- Behaviour over USB HID stays identical except for F-15, the unmount reset (§5.3) and the additive keys and ops. A non-W build has only the `usbHid` link, so the arbiter must always dispatch (ARB-01).
- Keep the exact poll order of `subscribe_poll()` and the rate/baseline semantics of HID_CONTROL_PLANE.md §7.
- The shared scratch buffers stay off the 4 KB core-0 stack.
- RAM increase on `pico2_pcm1808` ≤ 1 KB (BLD-01): report the before/after `RAM:` lines.

**Acceptance:** all host tests PASS (gcc + clang + sanitize), every firmware env builds, CI static analysis is clean on the changed lines (run `git clang-format --diff HEAD` and the CI cppcheck command from `.github/workflows/ci.yml`).
**Files:** new `include/control/{link,handler,session,arbiter,json_scan,control_handler}.h`, `src/control/{session,arbiter,arbiter_port,control_handler}.cpp`, `tests/test_control_session.cpp`, `tests/test_control_arbiter.cpp`; modified `src/hid_control.cpp`, `include/hid_control.h` (comments only), `src/hid_json.cpp`, `include/hid_json.h`, `tests/test_hid_json.cpp`, `tests/run_all.sh`, `tests/build_quartz_tests.sh` (if `test_hid_json` needs new sources).

### A3 — BLE link (sonnet), M1 then M2

**Context:** plan §2 (F-6..F-14), §4, §5.4. Framework sources: `~/.platformio/packages/framework-arduinopico/libraries/BLE/src/` (`BLECharacteristic.*`, `BLEServer.*`, `BLE.cpp`, `BLEServiceUART.cpp` as a cautionary example), `cores/rp2040/BluetoothLock.h`, `include/btstack_config.h`.
**Do (M1):**
- Write `frame_ring.h` and `ble_pacer.h` plus their tests: **RING-01..08** in `tests/test_frame_ring.cpp` (including a 2-thread stress test, linked with `-pthread`; add a per-entry flag hook to `run_all.sh` if needed) and **PACE-01..08** in `tests/test_ble_pacer.cpp`.
- Write `ble_control.h/.cpp` with echo mode (`TG_BLE_ECHO=1`), the INFO characteristic, advertising, connection-parameter request, and `ble_control_status()`.
- Add the CLI `ble` command inside the CLI sources, which only build with `TG_SERIAL_CLI` (§5.7). The primary status interface is `getLink`.
- Until A2 lands, echo builds can be checked with nRF Connect (HW-05) and the `pico2w_pcm1808_cli` env's `ble` command.
- Add `ble_control_begin()` / `ble_control_task()` calls in `TimegrapherStand_main.cpp`, guarded by `#if TG_BLE`.

**Do (M2, after A2 and gate G1):** replace echo with `BleLink` + `Session` + `ControlHandler`; implement `on_link_down` wiring and advertising resume.
**Constraints:**
- No `setValue()` on TX (F-7).
- Nothing in BT context calls into `Session`, `control_api` or `Serial`: BT context only touches the rings, atomics and the INFO buffer.
- The main loop makes no BTstack calls and never takes `BluetoothLock`. It reaches BT context only through the rings, atomics and `async_context_set_work_pending()` (§5.8 rules 1–3). The code must not assume which core BT context runs on.
- No heap after `setup()` (the library's own allocations in `begin()` are fine).
- `ble_control.cpp` compiles to nothing without `TG_BLE`.

**Acceptance:** host tests PASS (incl. sanitize). `pico2w`, `pico2w_synthetic`, `pico2w_pcm1808` and `pico2w_synthetic_noble` build. Non-W envs are byte-identical in behaviour (they don't compile BLE code). Report RAM/flash for `pico2w_pcm1808` (BLD-02).
**Files:** new `include/control/{frame_ring,ble_pacer}.h`, `include/ble_control.h`, `src/ble_control.cpp`, `tests/test_frame_ring.cpp`, `tests/test_ble_pacer.cpp`; modified `src/TimegrapherStand_main.cpp`, `src/cli/cli_commands.cpp`, `tests/run_all.sh`, `docs/build_and_flash.md` (echo override note only).

### A4 — Build targets, board config, CI (haiku)

**Context:** `platformio.ini`, `include/board_config.h`, `src/TimegrapherStand_main.cpp` (`led_set` and the `STATUS_LED_GPIO` uses), `.github/workflows/ci.yml`, plan §5.5, F-3 and F-4.
**Do:**
- Add the `[ble]` and `[board_pico2w]` sections and the 4 envs exactly as in §5.5.
- In `board_config.h`: `BOARD_PICO2W` → `STATUS_LED_GPIO -1`, `STATUS_LED_CYW43 1`, `TG_BOARD_NAME "Pico2W"`; also `#ifndef TG_BLE` → `0`. Order it before the `BOARD_PICO2` checks.
- In `led_set`: when `STATUS_LED_CYW43`, use `digitalWrite(LED_BUILTIN, …)`, with `pinMode(LED_BUILTIN, OUTPUT)` in `setup()`. Make the LED blink code compile for both.
- Add `pico2w_pcm1808`, `pico2w_synthetic` and `pico2_pcm1808_cli` to the CI firmware matrix.
- Add the `[cli]` section and the `pico2_pcm1808_cli` / `pico2w_pcm1808_cli` envs as **placeholders** with the §5.5 contents. A8 owns making the filter work.

**Acceptance:** **BLD-01..03** (all envs build; non-W unchanged; YAML valid: run `python3 -c "import yaml,sys;yaml.safe_load(open('.github/workflows/ci.yml'))"`). No BLE code is required: until A3 lands, `TG_BLE=1` just links the stack.
**Files:** `platformio.ini`, `include/board_config.h`, `src/TimegrapherStand_main.cpp` (LED only), `.github/workflows/ci.yml`.

### A5 — Docs and types (haiku)

**Context:** plan §4 (copy it), `docs/HID_CONTROL_PLANE.md`, `docs/AS_BUILT_ARCHITECTURE.md`, `docs/build_and_flash.md`, `docs/hid/timegrapher_hid.d.ts`, `docs/hid/webhid_example.md`.
**Do:**
- Create `docs/BLE_CONTROL_PLANE.md` (normative: §4 verbatim in structure, plus a "Host checklist" and a raw nRF Connect walkthrough using the hex frames in test HW-05).
- In `HID_CONTROL_PLANE.md`:
  - add a §1 note that the framing is transport-agnostic and BLE uses it (link);
  - add a new section "Control ownership" (plan §4.7, verbatim rules table);
  - add `hello.link` / `hello.control` / `caps.ble` / `caps.cli` to §5.1;
  - add `linkBusy` to §4.5;
  - document `claim`, `getLink`, `getDiag` and `controlReleased` in §5 and §7.2;
  - add H7 = BLE + ownership + CLI retirement to the phase table;
  - update §9 for the new module split;
  - change "CDC CLI remains available" to "the CLI is only in `TG_SERIAL_CLI` builds; the CDC port (upload auto-reset) is in every build".
- In `docs/DISPLAY.md`: the BT badge (also A7's job; whichever lands second reconciles).
- In `AS_BUILT_ARCHITECTURE.md`: the module table gains `control/*` and `ble_control`.
- In `build_and_flash.md`: the Pico 2 W envs, the GPIO25 warning, the `ble` CLI command, and the echo build override.
- In `timegrapher_hid.d.ts`: `HidHelloLink`, `HidHelloResult.link?`, `HidHelloCaps.ble?`, BLE UUID constants as `declare const` like the existing HID constants.

**Acceptance:** **DOC-01..04** (links resolve, the d.ts type-checks with `npx tsc --noEmit docs/hid/timegrapher_hid.d.ts`, UUIDs match §4.1 character for character).
**Files:** the listed docs only.

### A6 — `docs/hid/example.html` BLE option (sonnet)

**Context:** `docs/hid/example.html` (its framer and connect code near `navigator.hid`), plan §4 and §6.2.
**Do:**
- Add a transport selector, "USB (WebHID)" | "Bluetooth (Web Bluetooth)", that reuses the page's framer and request code through a small transport shim. The page has no build step: plain JS, no dependencies.
- Add a **"BLE probe"** panel that reports, for the current browser: `navigator.bluetooth` present, `getAvailability()`, `requestDevice` filter support, INFO contents, `writeValueWithResponse` / `writeValueWithoutResponse` availability, `startNotifications`, the `gattserverdisconnected` event, and a **throughput test**. The throughput test is `ping` ×100 sequentially (mean and p95 RTT), then `subscribe` at the maximum rates for 10 s, counting notifies per op, so it measures the delivered rates against the requested ones.
- Show the results in a copyable `<pre>`.

**Acceptance:** **EX-01..04**. Checked manually by the manager on desktop Chrome on Day 0 (WebHID path still works against an existing Pico 2), and on hardware day for BLE.
**Files:** `docs/hid/example.html` only.

### A7 — OLED Bluetooth badge (sonnet)

**Context:** `src/display_ui.cpp` (`draw_gps`, `draw_mode_tag`, `draw_header`, `draw_quartz_header`), `include/ui_snapshot.h`, `include/ui_format.h`, `tests/test_ui_format.cpp`, `docs/DISPLAY.md`, plan §5.6 and F-21.
**Do:**
- Add `ui_header_layout()` (pure, in `ui_format.h`), the badge drawing (`draw_ble`), the snapshot fields, and their publication in `publish_ui_snapshot()`. Read `ble_control_status()` behind `#if TG_BLE`; until A3-M2 lands, stub it to "enabled, not connected".
- Shorten the status strings per §5.6 (unless the user chose otherwise for D-UI).
- Add the `UNC` rule for the quartz header.
- Tests **UIH-01..08** in `tests/test_ui_format.cpp`; update the existing string expectations.

**Constraints:**
- Core 1 reads only the snapshot.
- No change to non-BLE layouts other than the shortened strings: the GPS and tag positions stay identical when `ble=false` (UIH-02).
- Keep the frame-hash skip working (the badge state is part of the frame, so a state change redraws).

**Acceptance:** host tests pass; all envs build; HW-17 on hardware day.
**Files:** `src/display_ui.cpp`, `include/ui_snapshot.h`, `include/ui_format.h`, `src/TimegrapherStand_main.cpp` (snapshot publish only), `tests/test_ui_format.cpp`, `docs/DISPLAY.md`.

### A8 — Retire the serial CLI (sonnet)

**Context:** `src/cli/*`, `src/serial_cli.cpp`, `include/serial_cli.h`, `src/diagnostics.cpp`, `include/diagnostics.h`, `include/app_config.h`, `platformio.ini`, `scripts/calibrate_timebase.py`, Adafruit `Adafruit_USBD_CDC.cpp` (F-18, read only), plan §5.7.
**Do:**
- Make the `[cli]` / `[base]` filter of §5.5 work, add the `serial_cli.h` inline stubs, and compile out the `diagnostics.cpp` print.
- Run the `Serial.` audit.
- Leave the CDC interface and `usb_audio.cpp` alone: CDC stays in every build (F-18, F-20).
- Write the CLI → HID parity appendix.
- Update `docs/build_and_flash.md`, and add a "legacy: needs a `*_cli` build" note to `calibrate_timebase.py`.
- Tests: **CLI-01..07**.

**Constraints:**
- Don't edit `hid_control.cpp` (A2 owns it; `getDiag` is A2's) or `usb_audio.cpp`.
- CLI builds (`*_cli`) must behave **exactly** as today, CDC included (CLI-03).

**Acceptance:** BLD-01 shows the flash and RAM savings (report the numbers); all envs build; host tests pass; CI matrix entry `pico2_pcm1808_cli` builds.
**Files:** listed in the §5.1 A8 row, plus `docs/build_and_flash.md` and `docs/HID_CONTROL_PLANE.md` (appendix only).

### A9 — Radio core selection `TG_BLE_CORE` (sonnet)

**Context:** plan §5.8 (all of it), F-12, F-17, F-22..F-27. Framework sources (read only): `variants/rpipico2w/`, `cores/rp2040/main.cpp`, `cores/rp2040/cyw43_wrappers.cpp`, `pico-sdk/src/rp2_common/pico_async_context/async_context_threadsafe_background.c`, `pico-sdk/src/rp2_common/pico_cyw43_driver/cyw43_driver.c`, `tools/platformio-build.py`. Project: `src/ble_control.cpp` (A3-M1, read; change only `ble_control_begin()`'s call site), `src/TimegrapherStand_main.cpp`, `platformio.ini`.
**Do:**
- Add `variants/rpipico2w_tg/` (§5.8), `include/ble_core.h` / `src/ble_core.cpp` (`ble_core_setup1()`, the radio-ready flag, the core-0 hold, the LED proxy of §5.8 rule 4, the core-1 stack paint and high-water mark).
- Change `TimegrapherStand_main.cpp`:
  - `setup1()`/`loop1()` for W builds without a display;
  - skip `multicore_reset_core1()` when `TG_BLE_CORE=1`;
  - call `ble_control_begin()` from `setup()` (core 0) or let `ble_core_setup1()` call it (core 1);
  - the hold after `usb_audio_init()`.
- Add `[ble_core1]` and env `pico2w_synthetic_core1` to `platformio.ini` (superseded 2026-11-23: folded into `[ble]` and `[board_pico2w]`, env removed). Add `getLink.ble.core` and `core1StackFree`: A9 gives A2 the two field names, and A2 encodes them.
- Tests: **CORE-01..05**, plus the HW-19 procedure written into `docs/build_and_flash.md`.

**Constraints:**
- Don't change the framework package. Everything lives in the project (variant copy, `board_build.*`).
- `TG_BLE_CORE=0` builds must produce the same code as before A9, apart from `getLink.ble.core` (the LED proxy is rule 4 and applies to both placements).
- If `init_cyw43_wifi()` from core 1 hard-faults or asserts (`assert(get_core_num() == …)` in the async context), stop and report the stack trace. Don't work around SDK asserts.
- No new heap after `setup1()` beyond what `init_cyw43_wifi()` and `BLE.begin()` allocate.

**Acceptance:** all envs build (core 1 is the default of the W envs). CORE-01..04 pass; CORE-05 and HW-19 are checked on hardware day. Report the boot time to "radio ready" on core 1 vs core 0 (from HW-01 logs).
**Files:** `variants/rpipico2w_tg/*`, `include/ble_core.h`, `src/ble_core.cpp`, `src/TimegrapherStand_main.cpp`, `platformio.ini`, `docs/build_and_flash.md` (HW-19 procedure section only), `tests/check_ble_core.sh` (static checks for CORE-02/04).

### W1 — `FramedTransport` base (sonnet)

**Context:** `timegrapher-web/src/transport/{Transport,WebHidTransport}.ts`, `WebHidTransport.test.ts`, `src/protocol/framer.ts`, plan §3.2.
**Do:**
- Extract the queue, single in-flight slot, ids, timeouts, late-reply ignore, notify dispatch, teardown, encoder and reassembler into `abstract class FramedTransport implements Transport`, with abstract hooks: `protected abstract openLink(): Promise<void>`, `protected abstract closeLink(): Promise<void>`, `protected abstract sendFrame(frame: Uint8Array<ArrayBuffer>): Promise<void>`, and protected `onFrame(data: DataView | Uint8Array)` / `onLinkLost(reason)` for subclasses to call.
- Make `WebHidTransport` extend it, keeping the exact public API, exports (`DEFAULT_REQUEST_TIMEOUT_MS`, `isWebHidSupported`) and behaviour.
- Add the keepalive (§6.2: `ping` after 3 s idle) and the `controlReleased` teardown in the base, so both transports get them.
- Add `FramedTransport.test.ts` covering **FT-01..13** with a trivial in-memory subclass. Existing `WebHidTransport` tests that count sent frames under fake timers may now see keepalive pings. If so, make the keepalive interval a constructor option and set it to 0 (off) in **those tests' construction only**; the test bodies stay unmodified.

**Acceptance:** the existing `WebHidTransport.test.ts` passes **unmodified**; lint, typecheck, test and build are all green.
**Files:** `src/transport/FramedTransport.ts`, `src/transport/FramedTransport.test.ts`, `src/transport/WebHidTransport.ts`.

### W2 — `WebBleTransport` + fake GATT + contract over BLE (sonnet)

**Context:** W1 output, plan §4 and §6.2, `tests/contract/*`, `src/sim/{VirtualTransport,VirtualDevice,index}.ts`.
**Do:**
- Implement `WebBleTransport` (§6.2), the constants, `TransportKind` `'webble'` and `ClientErrorCode` `'linkUnsupported'`. Add `@types/web-bluetooth` (exact version) and the tsconfig `types` entry if needed.
- Build a **fake Bluetooth stack** (`FakeBluetooth`, `FakeBluetoothDevice`, `FakeGattServer`, `FakeCharacteristic`). Its peripheral side wraps a `VirtualDevice`: it reassembles RX frames with the app's `FrameReassembler`, calls `device.handle()`, frames replies and `pollNotify()` output with `FrameEncoder` (reply cap 2048), and emits **unpadded** notification values. It's configurable for MTU, per-write latency, a "drop link" switch, and "fail next write".
- Write `WebBleTransport.test.ts` covering **WB-01..20**.
- Write `bleSimHarness.ts` + `contract.ble.test.ts` running the full `runTransportContract` suite (**CT-BLE-01..03**).

**Acceptance:** all new and old tests are green; coverage of `WebBleTransport.ts` ≥ 90 % lines (report it).
**Files:** the §6.1 W2 rows, plus `package.json` / `package-lock.json` and `tsconfig.json` (types only).

### W3 — Connection store, link profile, UI, iOS guidance (sonnet)

**Context:** `src/stores/{connection,subscriptions}.ts` + tests, `src/components/{ConnectPrompt,AppBarConnection}.vue`, plan §6.3 and §6.4.
**Do:**
- Factory branch `'webble'`. `webBleAvailable` (async availability plus the `availabilitychanged` listener). Label `"<board> · fw <v> · BLE"`. A snackbar for `linkUnsupported` that tells the user what to do.
- `LINK_RATE_CAPS` clamping in subscriptions.
- The exclusive-control flow of §6.3a: the takeover dialog, `linkBusy` / `controlReleased` handling, and `claim` / `hello.control` support in `VirtualDevice` (the simulator gets a `setForeignOwner(kind | null)` scenario hook so the flow is testable).
- `platform.ts`, then the UI per §6.4.
- Tests **CS-01..08**, **OWN-01..07**, **SUB-01..03**, **UI-01..06**, **PL-01..05**.

**Acceptance:** lint, typecheck, test and build all green. Screenshot-free manual check by the manager in desktop Chrome (BLE button present) and in an iPad UA override (guidance shown).
**Files:** the §6.1 W3 rows, plus `src/sim/VirtualDevice.ts` (claim / hello.control / foreign-owner hook only) and a new `src/components/TakeoverDialog.vue` (+ test).

### W4 — Web docs + type sync (haiku)

**Do:**
- Run `npm run sync-types` after A5 (no hand edits to `src/protocol/timegrapher_hid.d.ts`).
- In `README.md`: "Connecting over Bluetooth", covering desktop Chrome/Edge, Android Chrome, and iPad/iPhone via Bluefy with steps, the `TG-XXXX` naming, and the limits (foreground only, one BLE host at a time, no pairing).
- In `docs/ARCHITECTURE.md`: the layer diagram gets `FramedTransport` / `WebBleTransport`; add a Transport section for BLE.

**Acceptance:** **WDOC-01..02**; typecheck still green after the sync.
**Files:** `README.md`, `docs/ARCHITECTURE.md`, `src/protocol/timegrapher_hid.d.ts` (generated).

### 8.1 Manager review checklist (applied to every submission)

- [ ] Only the files in the brief changed (`git status --short` in both repos).
- [ ] Tests named in the brief exist, map 1:1 to spec IDs (an ID in the test name or a comment), and fail if the feature is removed (spot-check by mutation).
- [ ] Docs comments on every new symbol; no dead code or commented-out blocks; no TODOs without a test ID or issue reference.
- [ ] Firmware: no heap after setup; nothing in BT context touches non-atomic shared state; no BTstack call or `BluetoothLock` in `loop()` code (CORE-02); no `setValue()` on TX; no `BLE.startAdvertising()` after boot; no stack buffers > 256 B on core 0 or in BT context.
- [ ] Web: no `any`; single in-flight GATT operation; listeners removed on close; `DeviceError` codes as specified.
- [ ] Builds/tests re-run by the manager (not just trusted): `tests/run_all.sh` (also `SANITIZE=1`), `pio run` (all envs), CI cppcheck + clang-format commands, the Ubuntu container matrix via Colima for anything touching host tests, and the web `npm run lint && npm run typecheck && npm test && npm run build`.
- [ ] Kick back with numbered, concrete findings (file:line, what's wrong, what's expected) until clean.

---

## 9. Hardware day (2026-11-23)

### 9.1 Equipment

- 2 × Pico 2 W, plus the existing Pico 2 as the A/B reference. A PCM1808 front end or the synthetic source.
- iPad (USB-C) with **Bluefy** installed, and **nRF Connect** (iOS or Android) for raw GATT.
- Mac or PC with Chrome. An Android phone with Chrome is optional.
- A USB power bank (HW-12).
- Host recording setup per `tests/host_capture.md`, and `tests/host_analyze.py`.

### 9.2 Gates

| Gate | Build | Tests | Pass → | Fail → |
|---|---|---|---|---|
| **Baseline** | current committed firmware, `pico2_synthetic` on the existing Pico 2 (it still has the CLI) | HW-00 | — | — |
| **G0 bring-up** | `pico2w_synthetic_noble`, then `pico2w_synthetic_cli` | HW-01, HW-02 (noble vs BLE-advertising A/B), HW-18 (no-CLI build: CDC present, upload auto-reset) | G1 | LED/GPIO/PIO issue: fix A4/A3; if audio degrades with the radio merely on, escalate (R-1) |
| **G1 radio + link** | `pico2w_synthetic_cli` with `-DTG_BLE_ECHO=1` | HW-03, HW-04, HW-05, HW-06 (echo flood), HW-16 | A3-M2 | MTU < 66 on iPad/macOS: implement §4.5 (SEG tests). Audio glitches: §11 R-1 ladder |
| **G2 integration** | `pico2w_pcm1808` / `pico2w_synthetic` (M2, no CLI; diagnostics via `getDiag` / `getTiming` / `getLink`) + web app preview | HW-07..HW-15, HW-17, then HW-19 (radio core A/B), and HW-13 again on the chosen core | Done: finalise `LINK_RATE_CAPS` and `TG_BLE_CORE`, docs | Triage per failing test |

### 9.3 Recording results

Record every HW result in `docs/acceptance_results.md` under a new "BLE (Pico 2 W)" section: date, firmware git describe, env, client (device / OS / browser versions), measured values, pass/fail. The manager owns this file during hardware day.

---

## 10. Definition of done

1. Every test in `BLE_TRANSPORT_TESTS.md` not marked *conditional* or *optional* passes, and the HW results are recorded.
2. CI green in both repos (firmware: host tests ×3, firmware matrix including the W envs, static analysis, CodeQL; web: lint, typecheck, test, build).
3. Docs updated (A5, W4) and `LINK_RATE_CAPS` finalised from HW-07.
4. The manager summarises the changes per repo for the user to commit. Nothing is committed by agents.

---

## 11. Risks and mitigations

| ID | Risk | Likelihood | Detection | Mitigation |
|---|---|---|---|---|
| R-1 | CYW43/BTstack interrupt work on core 0 (F-17) causes USB audio underruns or DSP overruns | Medium | HW-02, HW-06, HW-13 (`diag` / `getDiag`, `dsp timing` / `getTiming`, `host_analyze.py`) | Ladder: (a) a longer BLE connection interval while USB audio is streaming (30–50 ms); (b) lower `LINK_RATE_CAPS`; (c) a time-sliced budget for `ble_control_task()` per loop; (d) **`TG_BLE_CORE=1`** (§5.8, A9, built in advance, compared in HW-19): the radio starts on core 1 next to the display, so core 0 only sees lock-free ring/flag operations |
| R-11 | *Retired in revision 3:* CDC stays, so the USB interface set doesn't change | — | — | — |
| R-12 | *Retired in revision 3:* upload auto-reset keeps working through CDC | — | HW-18 | — |
| R-14 | Core-1 radio (`TG_BLE_CORE=1`) runs into a cross-core problem: core 1 stack overflow (F-26), no free hardware alarm for core 1's alarm pool (F-22), the boot UUID print from core 1 racing TinyUSB on core 0 (F-12), or the custom variant drifting from the framework's | Medium | CORE-01..05, HW-19 | §5.8 rules: core-0 hold during radio start-up, a stack high-water check, the variant diff check (CORE-04). Fallback: `TG_BLE_CORE=0`, which needs no extra code |
| R-13 | A stale desktop WebHID tab holds control | Medium | HW-10 | The 10 s lease (closed tabs stop pinging) plus the explicit takeover (§4.7) |
| R-2 | Adafruit TinyUSB and BTstack conflict at runtime (both link, F-2) | Low–Medium | HW-01 | Investigate on G0; fall back to the Pico SDK TinyUSB device stack only if it's unavoidable (large change, re-plan) |
| R-3 | A must-have client negotiates MTU < 66 | Low on Apple, unknown on Android | HW-04 | §4.5 segmented mode (SEG tests) |
| R-4 | Bluefy lacks a Web Bluetooth feature we use (`writeValueWithResponse`, `getAvailability`, the `namePrefix` filter) | Medium | HW-16 probe | Feature-detect in `WebBleTransport`: write-mode fallback, a filter retry without `namePrefix`, availability assumed true when the API is missing |
| R-5 | iOS suspends Bluefy in the background, so the link drops | Certain | HW-11 | Documented. The app reconnect flow must stay clean (WB-11, CS tests) |
| R-6 | Flash writes (timebase save/log) stall XIP long enough to drop the BLE link | Low | HW-14 | The supervision timeout is 4 s, so it's unlikely. If seen, defer flash writes while BLE is connected and the host is idle |
| R-7 | The library's `prepareAdvertising()` prints the service UUID on CDC, confusing CLI scripts | Certain, benign | — | Printed once (F-27). Documented in `build_and_flash.md` |
| R-8 | No BLE security: anyone in range can connect | Certain | — | Documented. v2: an app-level PIN shown on the OLED |
| R-9 | PIO state-machine/instruction memory clash between CYW43 and the PCM1808 program | Low | HW-01 with `pico2w_pcm1808` | CYW43 claims first (F-23; with `TG_BLE_CORE=1` it claims in `setup1()`, possibly concurrently with `audio_source_init()`; claiming is spin-locked in the SDK). PCM1808 takes any free SM. RP2350 has 3 PIO blocks, so it fits; if needed, pin CYW43 to `pio2` via its config define |
| R-10 | Refactor A2 changes USB HID behaviour subtly | Medium | SES tests, existing suites, manual WebHID smoke with the existing Pico 2 | Behavioural invariant in A2's brief; manager mutation spot-checks |

---

## 12. Open items for the user

- **D-UI header wording (§5.6), applied as the default:** to fit the Bluetooth badge, three status strings get shorter (`no stream`, `clock check`, `listening`), and the quartz `UNCAL` badge becomes `UNC` when space is short. The alternative is to keep the wording and drop the `USB`/`SA` mode tag from the header on BLE builds. The default is the shortening.
- **Lease length (§4.7):** 10 s, with a keepalive every 3 s. A shorter lease makes a closed desktop tab release faster but costs more pings.
- **Name format:** `TG-XXXX`. Say so if you want a different prefix (it must keep the 31-byte advertisement budget, §4.2).
- **`ble off` persistence:** runtime-only, CLI builds only. A persistent HID `setLink {bleEnabled}` would need a `flash_store` field (not planned).

- **Radio core (§5.8):** decided on hardware day by the HW-19 rule (core 1 preferred if it passes). Nothing to decide now.

**Resolved in revision 2:** concurrency → exclusive (§4.7); the CLI is retired (§5.7).
**Resolved in revision 3:** the CDC port stays in every build and the `bootsel` op is dropped; the CPU division stays, apart from the radio, which becomes selectable (§5.8).

---

## 13. Detection pre-filter (revision 4)

### 13.1 Why

Today every filter in the DSP graph shapes only what USB plays. `dsp_graph_process()` runs two independent stages on the same raw block:

```
raw ─┬─► analyze():      levelMeter → matchedFilter → spectrum → noiseFloor → eventDetect → beatRate   (read-only)
     └─► process_path(): notch → filterBank → firBank → softGate → gain → agc → limiter → PCM24 → USB
```

So mains hum, handling rumble or hiss reach the tick detector unfiltered. The detector's open threshold follows the noise floor, so broadband noise raises the threshold and hides the quiet unlock sound, and low-frequency hum can open false events. The goal is to put the **linear** noise filters in front of the timing engine, without changing what the USB listener hears.

### 13.2 What goes in front of the detector, and what doesn't

| Measurement | Effect of filtering the detector input |
|---|---|
| Rate | Unaffected by any time-invariant filter: every tick is delayed equally |
| Beat error | Unaffected in practice: tick and tock see the same delay |
| Amplitude | **Sensitive**: it comes from the unlock → drop interval inside one tick (a few ms). A filter whose delay varies with frequency (IIR) smears those landmarks |

- **Allowed:** a **mains notch** (narrow, its phase effect at kHz tick content is a few µs) and a **linear-phase FIR band-pass** (constant delay at every frequency, so it shifts every landmark equally and the shift is compensated, §13.3).
- **Not allowed in the detection path:** `softGate`, `agc`, `limiter`, `gain`. They are level-dependent: they fight the detector's own threshold and noise tracking, and the gate swallows the quiet tick onset. The IIR `filterBank` (non-linear phase) is also left out.

### 13.3 Design (A10)

New analysis block **`detectFilter`** (`DSP_BLOCK_DETECT_FILTER`, appended before `DSP_BLOCK_COUNT` so existing IDs and saved state keep their values). It filters a copy of the block; the raw block still feeds USB processing, the level meter and the spectrum (they report what the pickup hears).

```
raw ─┬─► levelMeter(pre), spectrum                                        (raw, as today)
     ├─► detectFilter: notch(hz) → FIR band-pass(preset) ─► noiseFloor → matchedFilter → eventDetect → beatRate
     └─► process_path (unchanged) → PCM24 → USB
```

- **Own state.** Its own biquad and FIR instances and delay lines. It never shares state with the process-path `notch` / `firBank`, which keep working as today for the listener.
- **Parameters** (`patchState.blocks.detectFilter.params`, HID key `detectFilter`):
  - `notchHz`: `0` (off), `50` or `60`; default `0`. Reuses the existing notch design (`dsp_graph.cpp`).
  - `preset`: `0` bypass, `1` wide (160 Hz–16 kHz), `2` medium (400 Hz–12 kHz), `3` narrow (800 Hz–8 kHz); default `1`. Reuses `design_fir()`'s window method, but with **65 taps** (odd, so the group delay is a whole **32 samples = 0.667 ms** at 48 kHz).
- **Default `enabled: false`.** With the block off, the detector input is the raw block and results must be bit-identical to today (DPF-01).
- **Delay compensation.** With the FIR active, the detector is called with `base - 32` (saturating at 0 right after a stream reset; the first 64 samples after a reset are filter warm-up and must not open an event). Every event index (`onset`, `unlock`, `drop`, `peak`) then refers to the raw sample time, so absolute timestamps (beats notify, timebase-disciplined timing) don't move when the filter is toggled.
- **Toggle behaviour.** Enabling, disabling or changing a parameter resets the filter state and the detector's in-progress event (not the beat history), so no event mixes filtered and unfiltered samples.
- **Pipeline graph** (`dsp_blocks.cpp`, `hello.caps.pipeline`): new node `detectFilter` (kind `analysis`, inputs `{input}`); `noiseFloor`, `matchedFilter` and `eventDetect` take `detectFilter` as input instead of `input`. `HID_CONTROL_PLANE.md` table and `timegrapher_hid.d.ts` updated.
- **Cost.** A 65-tap FIR over 240 samples is about 15.6 k multiply-adds per 5 ms block, plus one biquad. Measured on hardware with `getTiming` (DPF-09); the budget is 5000 µs per block.

### 13.4 Matched-filter fixes (A11)

Found while reviewing the graph (2026-11-23). The matched filter learns a 64-sample template from accepted ticks and **vetoes the start of a tick event in any 5 ms block whose score is below the threshold** (`tick_detect.cpp`, `open_veto`). Its score also blends into the event-log `confidence`, which nothing else reads. Three defects:

1. **The score depends on loudness.** The template is unit-energy but the window isn't normalized, so the score is the projection *amplitude*, not a 0–1 shape match. A quiet pickup (ticks around 0.01–0.05 FS) never reaches the 0.35 default, and once a template is learned every block is vetoed: detection stops. Fix: normalized cross-correlation (divide by the window's energy as well), so the score is in [0, 1] and the threshold means "shape agreement".
2. **The veto covers a whole block.** It is decided once per block from the block's best position. Fix: the detector consults the score at the candidate onset sample (a per-sample score buffer, or evaluate the correlation only at candidate onsets, which is also far cheaper than the current 241-position sweep).
3. **Self-reinforcing learning.** It learns only from events it let through; if it ever accepts noise, it drifts towards noise. Fix: learn only from events the beat analysis accepted as beats (they are on the locked beat grid), and stop learning (`learn` off) once the template has converged (correlation between successive updates > 0.98 for 20 events), until a reset.

The matched filter keeps reading the **detectFilter** output (§13.3), so its template is learned from the same signal the detector sees.

### 13.5 Synthetic impairments (A12)

To test §13.3–13.4 without a PCM1808, synthetic builds (`AUDIO_SOURCE == SYNTHETIC`) get optional impairments mixed into the loop **before** the DSP graph and the USB ring:

- `humHz` (`0`, `50`, `60`) and `humAmp` (FS, with 2nd and 3rd harmonics at −6 / −12 dB): mains hum.
- `noiseAmp` (FS RMS): white noise from a fixed-seed PRNG (reproducible).
- `clickRate` (per second) and `clickAmp` (FS): random single-sample clicks off the beat grid, which must not be counted as ticks.

New HID op `setSynth { humHz, humAmp, noiseAmp, clickRate, clickAmp }` / `getSynth`, present only when `hello.caps.synth` is true (synthetic builds). Runtime-only, never saved. Defaults all 0 (clean loop, as today). The USB stream carries the impaired signal too, so the listener hears what the detector gets.

### 13.6 Timing statistics clarity (A13)

- `getTiming` `*Mean` is an exponential average with weight 1/8 per DSP block (≈ 40 ms memory at 5 ms blocks); `*Max` is a high-water mark. Document both in `timegrapher_hid.d.ts` and `HID_CONTROL_PLANE.md`.
- Today the maxima are reset only by `resetTiming` and by the CLI diagnostics print, so on no-CLI builds a boot-time outlier stays forever. Add a rolling maximum over a fixed **5 s window** (`dspBlockUsMax5s`, `loopUsMax5s`, `quartzBlockUsMax5s`), keeping the existing fields as all-time maxima since the last reset.
- Add `loopBusyPct` breakdown fields where they are cheap: `dspUsPerS` (sum of DSP block times over the last 1 s window), so DSP load can be told apart from other loop work (BLE on core 0, HID, display hand-off).

### 13.7 Web app (W5)

- DSP view: the `detectFilter` block appears from `hello.caps.pipeline` / `getState` like the other blocks; its controls are a notch selector (Off / 50 / 60 Hz) and a band preset selector. The graph view shows the two branches (detection and listening).
- Settings → Simulator (only when `caps.synth`): sliders for the §13.5 impairments.
- `VirtualDevice` models `detectFilter` (state and params only; its signal model doesn't need the filter) and `caps.synth` with `setSynth` / `getSynth` state.
- Timing panel: show the 5 s maxima and label the mean as "≈ 40 ms average".

### 13.8 Tasks

| Task | Model | Depends on | Scope |
|---|---|---|---|
| A10 detection pre-filter | sonnet | — | §13.3 in `dsp_graph.cpp` / `dsp_blocks.cpp` / `dsp_types.h`, delay compensation in the `tick_detect_block` call, host tests DPF-01..07 |
| A11 matched-filter fixes | sonnet | A10 | §13.4, host tests MF-01..05 |
| A12 synthetic impairments | haiku | — | §13.5 in `audio_source.cpp` + HID op + caps, host test SYN-01..03 |
| A13 timing statistics | haiku | — | §13.6 in `perf_timing.cpp` / `hid_json.cpp` + docs |
| W5 web app | sonnet | A10, A12, A13 (types) | §13.7, plus `timegrapher_hid.d.ts` sync |

Acceptance on hardware with the synthetic source (no PCM1808 needed): DPF-08..10 and SYN-04 in [`BLE_TRANSPORT_TESTS.md`](BLE_TRANSPORT_TESTS.md) §11.

### 13.9 Open items for the user

- **Default for `detectFilter`:** planned `enabled: false` (no behaviour change on upgrade). Once DPF-10 passes on real pickups, consider defaulting it on with `notchHz` 0 and `preset` 1 (wide).
- **Matched filter:** keep it (with the §13.4 fixes) or retire it. With a working pre-filter, its value is click rejection only.

---

## 14. BLE throughput for fast movements (revision 5)

### 14.1 Why

Measured on hardware (HW-07, macOS): the BLE link carries about **100 notifications per second**, each one frame of 5 header bytes + 58 JSON bytes, so ≈ 5.8 KB/s of JSON. A `beats` notify spends ≈ 200 JSON bytes per beat. A 36,000 bph movement makes 10 beats per second, so `beats` alone needs 36–50 frames per second, and together with a 1 Hz `beat` summary and 2 Hz `metrics` the link is at ≈ 75 % of capacity. The three changes below act on the three biggest factors: bytes per beat, bytes per notification, and notifications per second. Target: 36,000 bph with every beat delivered, `beat` at 2 Hz and `metrics` at 5 Hz, under 40 % of link capacity on macOS.

### 14.2 Packed beats (A14)

**Stays JSON.** The envelope is unchanged; the beats travel as one base64 string under one key:

```json
{"v":1,"notify":true,"op":"beatsPacked","data":{"p":"AQgrCAAA…"}}
```

- **Opt-in.** `subscribe {"beatsPacked": true}` (new `HidSubscription` field). It replaces the JSON `beats` stream for that session: when both are set, only `beatsPacked` is sent. Same cursor rules as `beats` (oldest unsent first, lost beats skipped when the ring wraps).
- **History.** `getBeats {"max": n, "packed": true}` returns `{"p": "<base64>"}` with the same layout, newest **last** (a batch is always in seq order).
- **Capability.** `hello.caps.beatsPacked: 1` (the format version below). Absent = not supported.
- **Encoding.** Standard base64 (RFC 4648 §4, with `=` padding) of a little-endian binary block.

**Layout, format 1** (all little-endian; `i` = signed, `u` = unsigned):

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `fmt` | 1 |
| 1 | u8 | `n` | beats in the batch, 1..32 |
| 2 | u32 | `seq0` | seq of the first beat |
| 6 | u32 | `gridId` | grid of every beat in the batch |
| 10 | f32 | `nominalUs` | nominal half-period, µs |
| 14 | u64 | `sample0` | `sampleIndex` of the first beat |
| 22 | i32 | `k0` | `k` of the first beat |
| 26 | n × 17 B | beats | below |

Per beat (17 bytes):

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `dSeq` | seq − previous seq (0 for the first beat) |
| 1 | u8 | `flags` | bit 0 parity, bit 1 amplitudeValid, bits 2–3 status (0 accepted, 1 started, 2 rejected) |
| 2 | u32 | `dSample` | sampleIndex − previous sampleIndex (0 for the first) |
| 6 | u8 | `frac` | fractional unlock sample × 256 (`tS = (sampleIndex + frac/256) / sourceHz`) |
| 7 | i16 | `dK` | k − previous k (0 for the first) |
| 9 | i32 | `u` | residual, 0.1 µs units |
| 13 | u16 | `amp` | amplitude, 0.01° units (0 when invalid) |
| 15 | u16 | `lift` | lift time, µs (saturates at 65,535) |

- **Batch breaks.** The encoder ends a batch (and the next notify starts a new header) when the grid or the nominal period changes, when a delta does not fit its field (`dSeq` > 255, `dSample` ≥ 2³², `dK` outside i16), or when the base64 would not fit the 2048-byte message. So a decoder never needs grid-change logic inside a batch.
- **Size.** 8 beats: 26 + 8 × 17 = 162 bytes → 216 base64 characters; with the envelope ≈ 280 bytes, **5 frames instead of 29** (63-byte frames) or 2 frames with §14.3.
- **`sourceHz`** for `tS` comes from `getBeat.sourceHz` (unchanged while the stream runs).

### 14.3 Large TX frames on BLE (A15)

The frame format is unchanged (5-byte header; `payloadLen` is a u8). Today every frame is ≤ 63 bytes because of HID's report size; on BLE the ATT MTU allows ≈ 524. Larger frames cut the per-notification overhead and the notification count.

- **Measured limit (2026-11-23):** a notification value above 248 bytes (HCI ACL payload > 255) stalls the CYW43 radio on this stack until reset, so the Pico 2 W caps TX frames at **244** (`txMax`), not 255.
- **Opt-in, per connection.** A BLE client sends `hello {"frameMax": N}` (any `hello`; absent = keep the current value). The device answers that `hello` with the old frame size, then uses `txFrame = min(N, 255, attMtu − 3)` bytes for every later TX frame on that link (chunk = `txFrame − 5`, ≤ 250). The `hello` result's `link` object reports `frameMax` (the value in use). It resets to 63 when the link drops.
- **RX is unchanged:** the device still accepts only ≤ 63-byte frames from the client (`FrameReassembler` limits stay), so its RX ring keeps 64-byte slots.
- **Old clients** never send `frameMax` and never see a frame above 63 bytes: no version change is needed. USB HID ignores `frameMax` (fixed 63-byte reports).
- **INFO** gains `"txMax":255` (the largest `frameMax` the build accepts); clients that see it may opt in.
- **Firmware:** `FrameRing` takes its slot size as a template parameter; the BLE TX ring becomes 16 × 256 B (+3 KB RAM); `FrameFragmenter::begin()` takes the chunk size; `Session` holds the per-link TX frame size and reads `frameMax` from a reassembled `hello` (via `json_scan`), bounded by a new `Link::max_tx_frame()` (63 by default; `BleLink`: `min(255, mtu − 3)`). A frame larger than the pacer's ATT limit is still dropped and counted (`txOversize`), so a late MTU drop can't corrupt a frame.

### 14.4 15 ms connection interval with fallback (A15)

- One second after connect, request **min = max = 15 ms** (12 units), latency 0, timeout 4 s. Apple's rules allow min = max = 15 ms.
- Listen for the central's answer: `L2CAP_EVENT_CONNECTION_PARAMETER_UPDATE_RESPONSE` (result ≠ 0 = rejected) and `HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE` (the interval actually chosen). On a rejection, request the previous range **15–30 ms** once. On a second rejection, keep what the central chose. A central may accept and still pick another interval; that is not retried.
- `getLink.ble` gains `connParamStep` (`0` none yet, `1` asked 15 ms, `2` asked 15–30 ms, `3` gave up) and `connParamRejects`; `connIntervalUs` already reports the result.

### 14.5 Web app (W6)

- **Packed beats:** a decoder (`src/protocol/beatsPacked.ts`) that turns a `beatsPacked` notify or a packed `getBeats` into the existing `DspBeat[]` objects, so the stores and charts don't change. Subscribe with `beatsPacked: true` instead of `beats: true` whenever `caps.beatsPacked` is present, on **both** links.
- **Large frames:** `WebBleTransport` sends `hello {frameMax: 255}` right after `startNotifications()` when INFO has `txMax`, and its frame check accepts values up to the negotiated size. The shared reassembler accepts `payloadLen` up to 250.
- **Rate caps:** `LINK_RATE_CAPS.webble` revised after HW-07b (§14.6).
- `VirtualDevice` supports `beatsPacked` (subscribe, notify, `getBeats packed`) and `frameMax`; `timegrapher_hid.d.ts` synced.

### 14.6 Acceptance

Host tests BP-01..06 and LF-01..05, hardware HW-07b and HW-20 in [`BLE_TRANSPORT_TESTS.md`](BLE_TRANSPORT_TESTS.md) §12. HW-20 is the 36,000 bph case, run with the synthetic loop played at double speed (a test-only `TG_SYNTH_SPEED=2` build flag).

