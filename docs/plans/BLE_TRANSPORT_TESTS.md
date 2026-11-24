# Test specification: BLE control-plane transport

Companion to [`BLE_TRANSPORT_PLAN.md`](BLE_TRANSPORT_PLAN.md). Every test has an ID, an owner task, inputs/steps, and an exact expected result. Agents implement the automated tests **with the ID in the test name** (C++: in the `check()` label or a comment directly above the case; Vitest: `it('WB-07 …')`), so the manager can trace each requirement to its test.

**Conventions**

- *Firmware host tests* are plain C++17 executables using `assert()` / a local `check(cond, "ID: text")` helper, in the style of the existing `tests/test_*.cpp`. No test framework. They must run under `tests/run_all.sh` with gcc and clang, and with `SANITIZE=1`.
- *Web tests* are Vitest (happy-dom), colocated `*.test.ts` or in `tests/contract/`.
- "Frame" means the 63-byte-max buffer **without** a report ID: `[flags, msgSeq, fragIndex, fragCount, payloadLen, chunk…]`.
- Helper notation: `F(flags, seq, idx, cnt, "text")` builds a frame with `payloadLen = strlen(text)`. `Fpad(...)` is the same zero-padded to 63 bytes.
- Time is passed explicitly (`now_ms`). No test sleeps.
- *Conditional* = implement only if the plan's gate triggers it. *Optional* = nice to have, not part of done.

---

## 1. Firmware: framing (`tests/test_control_frame.cpp`), owner A1

### 1.1 Reassembler (FR-RX)

| ID | Case | Steps | Expected |
|---|---|---|---|
| FR-RX-01 | Single-fragment message | `push(F(0,7,0,1,"{\"a\":1}"), t=0)` | `kComplete`; `ready()`; `message()=="{\"a\":1}"`; `length()==7` |
| FR-RX-02 | Shorter than header | `push(buf, len=4)` while a 3-fragment message has fragment 0 stored | `kShort`; the partial message survives (fragment 1, 2 then complete it) |
| FR-RX-03 | `fragCount == 0` | `push(F(0,1,0,0,"x"))` after storing a partial | `kInvalid`; the partial is discarded (a later fragment of the old seq can't complete it) |
| FR-RX-04 | `fragIndex ≥ fragCount` | `F(0,1,3,3,"x")` | `kInvalid`, state cleared |
| FR-RX-05 | `payloadLen > 58` | header with `payloadLen=59`, buffer 64 bytes | `kInvalid` |
| FR-RX-06 | `fragCount` 36 rejected, 35 accepted | `F(1,1,0,36,58×'a')` / `F(1,2,0,35,58×'a')` | 36 → `kInvalid`; 35 → `kAccepted` |
| FR-RX-07 | Buffer shorter than `5+payloadLen` | header says 10, buffer has 5+9 bytes | `kShort`, state untouched |
| FR-RX-08 | Multi-fragment in order | 3 fragments (58, 58, 10 bytes), same seq | `kAccepted`, `kAccepted`, `kComplete`; `length()==126`; content byte-exact |
| FR-RX-09 | Out of order | same as 08, pushed in order 2, 0, 1 | `kComplete` on the last push; content byte-exact (offset = idx × 58) |
| FR-RX-10 | Duplicate `fragIndex` | push idx 0 ("AAA…"), push idx 0 again with different bytes ("BBB…"), push idx 1 | second push → `kDuplicate`; completed message contains the **first** copy |
| FR-RX-11 | New `msgSeq` abandons partial | seq 5 idx 0 of 2, then seq 6 idx 0 of 1 | seq 6 → `kComplete`; message is seq 6's text |
| FR-RX-12 | Same seq, different `fragCount` | seq 5 idx 0 cnt 3, then seq 5 idx 1 cnt 2, then seq 5 idx 2 cnt 3 | second → `kCountMismatch` (state cleared, frame dropped); third starts a new reassembly (`kAccepted`) that can't complete |
| FR-RX-13 | Timeout boundary: 500 ms accepted | idx 0 at t=1000, idx 1 (last) at t=1500 | `kComplete` |
| FR-RX-14 | Timeout boundary: 501 ms restarts | idx 0 at t=1000, idx 1 at t=1501 | the second push resets, then starts a new message with that frame (idx 1 of 2) → `kAccepted`, not complete |
| FR-RX-15 | `poll_timeout` | idx 0 at t=0; `poll_timeout(501)`; then idx 1 at t=502 | after poll, state empty; idx 1 alone → `kAccepted` (not complete) |
| FR-RX-16 | `poll_timeout` doesn't touch a ready message | complete at t=0; `poll_timeout(10000)` | still `ready()` |
| FR-RX-17 | Busy while ready | complete message, then push any valid frame | `kBusy`; the message is unchanged |
| FR-RX-18 | Same seq after release is a new message | complete seq 9; `release()`; push the same frame again | `kComplete` again (HID_CONTROL_PLANE §3.4 "re-executed") |
| FR-RX-19 | Max request size | 35 fragments × 58 = 2030 bytes | `kComplete`; `length()==2030`; NUL at [2030] |
| FR-RX-20 | Padded vs unpadded equivalence | the same 3-fragment message as `F` (unpadded) and as `Fpad` (63 bytes) in two reassemblers | identical `message()` and `length()`; bytes after `5+payloadLen` ignored |

### 1.2 Fragmenter (FR-TX)

| ID | Case | Steps | Expected |
|---|---|---|---|
| FR-TX-01 | Empty message | `begin("",0,false)`; `peek(out,false)` | returns 5; `out=[0,1,0,1,0]`; after `commit()` → `!active()` |
| FR-TX-02 | First message seq is 1 and increments | three messages | seqs 1, 2, 3 |
| FR-TX-03 | Seq wraps | 256 messages | message 256 has seq 0, message 257 seq 1 |
| FR-TX-04 | Chunk boundaries | lengths 58, 59, 116, 117 | fragment counts 1, 2, 2, 3; `payloadLen` 58 on all but the last; `more` flag on all but the last |
| FR-TX-05 | Clamp at 2048 | `begin(buf, 3000, …)` | 36 fragments; total payload 2048; last fragment `payloadLen` 18 |
| FR-TX-06 | Notify hint | `begin(json, n, true)` | every fragment `flags & 0x02`; the last fragment `flags == 0x02` |
| FR-TX-07 | **Peek is idempotent (F-15 regression)** | 3-fragment message; `peek` twice without `commit` | identical bytes and length both times; after `commit`, `peek` gives fragment 1 |
| FR-TX-08 | Pad policy | `peek(out,true)` vs `peek(out,false)` on a 10-byte last chunk | 63 vs 15 bytes; padded bytes 15..62 are zero |
| FR-TX-09 | `begin` while active | `begin(A)`; `begin(B)` | second returns false; peek still yields A's fragment 0 |
| FR-TX-10 | `abort` | `begin`; `abort()` | `!active()`; next `begin` works and seq continues (+1) |

### 1.3 Round trip (FR-RT)

| ID | Case | Expected |
|---|---|---|
| FR-RT-01 | Every length 0..2030: random bytes 0x20..0x7E → fragmenter (unpadded) → reassembler | byte-exact equality for every length |
| FR-RT-02 | Multi-byte UTF-8 (`"°±µ"` repeated) split across chunk boundaries, lengths 1..400 | byte-exact; no fixup of split code points (the framing is byte-oriented) |
| FR-RT-03 | Padded fragments → reassembler, lengths {0,1,57,58,59,2030} | byte-exact |

### 1.4 HID report-ID normaliser (FR-HID, `include/control/hid_report.h`)

| ID | Case | Expected |
|---|---|---|
| FR-HID-01 | `report_id = 1`, buffer = frame | `ok`, the pointer and length are unchanged |
| FR-HID-02 | `report_id = 0`, `buf[0] == 1` | `ok`, pointer +1, length −1 (today's "some stacks leave report ID" rule) |
| FR-HID-03 | `report_id = 2` | `!ok` (ignored) |

---

## 2. Firmware: session (`tests/test_control_session.cpp`), owner A2

Fakes: `FakeLink` (settable `connected`, `can_send`, a `send_ok` scripted queue, `pad`, a recorded frame list) and `FakeHandler` (records requests, replies with a scripted JSON per request, can queue notifies for `poll`, counts `reset()`). Unless noted, `max_frames_per_task = 1`.

| ID | Case | Expected |
|---|---|---|
| SES-01 | One request → one reply | the handler sees the exact JSON once; the link gets the reply's frames in order with consecutive `fragIndex`, one per `task()` |
| SES-02 | Request B arrives while A's reply is still being sent | A's reply is sent completely first; then B is dispatched (not dropped); B's reply follows |
| SES-03 | Request C arrives while B is complete but not yet dispatched (A still sending) | C's frames → `kBusy`; the handler never sees C |
| SES-04 | Backpressure | `can_send=false` for 5 tasks mid-message, then true | no frames during the stall; afterwards they resume with the next `fragIndex`; nothing skipped or duplicated |
| SES-05 | **Send failure retries the same frame (F-15)** | `send_ok` = [true, false, true, true] on a 3-fragment reply | the link receives frames idx 0, 1 (failed), 1, 2; the delivered bytes of idx 1 are identical both times |
| SES-06 | Notifies never interleave | a notify is queued while a 3-fragment reply is sending | all reply fragments come before the notify's first fragment; the notify has the next `msgSeq` |
| SES-07 | `poll` gating | the TX is busy, or the link is disconnected | `poll()` is not called |
| SES-08 | Link down | partial RX stored, TX mid-message; `on_link_down()` | `reset()` called once; TX inactive; RX cleared; after reconnect a new request works and the reply `msgSeq` continues incrementing |
| SES-09 | Automatic link-down detection | `connected` goes true → false between tasks | the session calls `on_link_down` itself (once per transition) |
| SES-10 | Two sessions are isolated (framing level) | sessions S1 (link L1, handler H1) and S2 (L2, H2); a request on L1; a notify queued in H2 | the reply appears only on L1; the notify only on L2; the msgSeq counters are independent (ownership is tested in ARB-*) |
| SES-11 | Reassembly timeout via `task(now)` | fragment 0 of 2 at t=0; `task(501)`; fragment 1 at t=502 | the handler is never called |
| SES-12 | Pad policy | HID-like link (`pad=true`) vs BLE-like (`pad=false`) | frames are 63 bytes vs `5+payloadLen` |
| SES-13 | `max_frames_per_task` | BLE-like link with limit 4, 10-fragment reply, `can_send` always true | 4, 4, 2 frames over three tasks |
| SES-14 | Handler sees the link | the handler calls `out.link().kind()` inside `on_request` | returns `"usbHid"` / `"ble"` per the fake link |
| SES-15 | Request not starved by notifies | BLE-like link, limit 4; the handler always has a 10-fragment notify due; a request arrives mid-stream | the request is dispatched within 3 tasks. Regression for the hardware finding of 2026-11-23 (max-rate subscription: `unsubscribe` never answered) |

### 2.1 `hello` encoder additions (`tests/test_hid_json.cpp`), owner A2

| ID | Case | Expected |
|---|---|---|
| HJ-01 | `hello` with link kind `usbHid` | valid JSON; `"link":{"kind":"usbHid"}`; no `mtu` key |
| HJ-02 | `hello` with link kind `ble`, mtu 185 | `"link":{"kind":"ble","mtu":185}` |
| HJ-03 | `caps.ble` / `caps.cli` only when built | the test compiles the encoder twice (`-DTG_BLE=0/1`, `-DTG_SERIAL_CLI=0/1`, or with the values passed as parameters) and checks absent / `true`; the hello, at its longest, still fits the size budget the existing test asserts |
| HJ-04 | `hello.control` | owner `ble`, self false → `"control":{"owner":"ble","self":false,"leaseMs":10000}`; owner none → `"owner":null` |
| HJ-05 | `getLink` result | with and without the `ble` object; every key of §4.6 present; valid JSON; the longest values (name `TG-FFFF`, the counters at `UINT32_MAX`) fit within 2048 with the envelope |
| HJ-06 | `getDiag` result | every `diagnostics_t` field present as camelCase; the maxima fit |

### 2.2 Ownership arbiter (`tests/test_control_arbiter.cpp`), owner A2

Pure `control::Arbiter`. `U` = usbHid, `B` = ble; `t` in ms. "Dispatch" = `Decision::kDispatch`.

| ID | Case | Expected |
|---|---|---|
| ARB-01 | Single-link device | only U ever asks: 1000 mixed ops over 1 h of virtual time, with gaps up to 60 s | every op dispatches (or `kAnswerLocally` for hello/ping/claim); never `kBusy`; `reset_handler` never set |
| ARB-02 | Auto-claim from none | `getState` from U at t=0 | dispatch; owner U |
| ARB-03 | Non-owner refused | then `getState` from B at t=1000 | `kBusy`; owner still U |
| ARB-04 | hello / ping always answered | `hello`, `ping` from B while U owns | `kAnswerLocally`; owner U; U's lease unchanged |
| ARB-05 | Owner ping refreshes the lease | U op at 0, U ping at 9000, B op at 15000 | B → `kBusy` (the lease runs to 19000) |
| ARB-06 | Non-owner ping doesn't refresh | U op at 0, B ping at 9000, B op at 10001 | B → dispatch (U's lease expired at 10000); `reset_handler` true; `notify_released == U` |
| ARB-07 | Lease boundary | U op at 0; B op at exactly 10000 → `kBusy`; at 10001 → dispatch | boundary is `now − deadline > 0` (signed wrap-safe) |
| ARB-08 | Expiry alone resets nothing | U op at 0; nothing until 60000; U op at 60000 | dispatch; `reset_handler` false (still U's) |
| ARB-09 | claim when free | B `claim` at 0 | `kAnswerLocally`; owner B; no reset |
| ARB-10 | claim when busy, no takeover | U owns; B `claim` | `kBusy` |
| ARB-11 | claim with takeover | U owns (lease valid); B `claim takeover` | `kAnswerLocally`; owner B; `reset_handler` true; `notify_released == U` |
| ARB-12 | Takeover by the owner itself | U owns; U `claim takeover` | owner U; `reset_handler` **false**; no release notify |
| ARB-13 | Owner link down | U owns; `on_link_down(U)` | returns true (reset); owner none; next B op dispatches without reset |
| ARB-14 | Non-owner link down | U owns; `on_link_down(B)` | returns false; owner U |
| ARB-15 | millis() wrap | U op at `0xFFFFF000`; B op at `0x00000800` (lease not yet expired across the wrap) | `kBusy`; at `0x00002000` + 10000 → dispatch |
| ARB-16 | Port integration (with the session fakes) | two Sessions with fake links, real `Arbiter`, fake `ControlHandler` | a B request while U owns gets a `linkBusy` error reply **with the request's id**; after takeover, U's session receives exactly one `controlReleased` notify once its TX is idle; `poll()` pushes go only to the owner session |

---

## 3. Firmware: BLE pieces (owner A3)

### 3.1 SPSC ring (`tests/test_frame_ring.cpp`)

| ID | Case | Expected |
|---|---|---|
| RING-01 | Empty pop | `pop` → false; `size()==0` |
| RING-02 | Push/pop round trip | a 31-byte frame | identical bytes and length |
| RING-03 | Capacity | `FrameRing<8>`: 8 pushes OK, the 9th fails | `overflows()==1`; 8 pops in FIFO order |
| RING-04 | Oversize push | len 65 | rejected; counted as overflow (or a separate `rejected()`, which must be documented); the ring is unchanged |
| RING-05 | Zero-length frame | len 0 | accepted and popped with len 0 (the ring is content-agnostic) |
| RING-06 | Wraparound | 10 × capacity cycles of push/pop with a sequence-stamped payload | order and content preserved; index wrap of `uint16_t` exercised (≥ 70 000 total operations) |
| RING-07 | `clear()` | 3 pushed, `clear()` | `size()==0`; the next push/pop works |
| RING-08 | Threaded SPSC stress | a producer thread pushes 1 000 000 frames (seq in the first 4 bytes, the length varies 1..63, the content derives from seq), retrying on full; the consumer pops and verifies | every seq exactly once, in order, content correct; test time < 10 s; runs clean under ASan/UBSan (and TSan locally if available) |

### 3.2 Notify pacer (`tests/test_ble_pacer.cpp`)

Fakes: a `request_can_send` counter, a `notify` recorder with a scripted return status (0 = OK, non-zero = BTstack error such as `BTSTACK_ACL_BUFFERS_FULL`), and the TX ring.

| ID | Case | Expected |
|---|---|---|
| PACE-01 | Push + `kick()` when idle | exactly 1 `request_can_send` |
| PACE-02 | A second push + `kick()` while pending | still 1 request (no duplicate registration) |
| PACE-03 | `on_can_send` with 2 frames queued | sends frame 1, pops it, re-registers (2 requests total); next `on_can_send` sends frame 2 and does **not** re-register |
| PACE-04 | Notify error | the status is non-zero on the first `on_can_send` | the frame stays queued; re-registered; the next `on_can_send` sends the same frame; `tx_retries==1` |
| PACE-05 | `on_can_send` with an empty ring | no notify, no re-register, pending cleared |
| PACE-06 | Disconnect | 3 queued, pending; `on_disconnect()` | ring empty, pending false; a `kick()` with no connection does nothing |
| PACE-07 | `con_handle` passed through | notify is called with the connection handle given at connect | handle equality |
| PACE-08 | CCCD off | `can_send()` false; `kick()` doesn't register |
| PACE-11 | BTstack delivers the can-send callback inside the registration call (controller has a free buffer) | Not left pending; later frames are sent. Regression for the hardware stall of 2026-11-23: one notification per connection, hello timed out |

### 3.3 Segmented mode (*conditional*: only if gate G1 triggers §4.5)

| ID | Case | Expected |
|---|---|---|
| SEG-01 | Segment a 63-byte frame at `mtu=23` (19 data bytes per value) | 4 values; the first has START; the payloads concatenate to the frame |
| SEG-02 | Reassemble the segments | the frame is emitted once, byte-exact |
| SEG-03 | Non-START value with an empty accumulator | dropped; counted |
| SEG-04 | START in the middle of a frame | the old partial is discarded, a new frame starts |
| SEG-05 | Web encoder/decoder equivalence | the TS segmenter output is accepted by the C++ reassembler (shared golden vectors in `tests/vectors/ble_seg_*.hex`) |

---

### 3.4 OLED header layout (`tests/test_ui_format.cpp`), owner A7

`ui_header_layout(pps, ble, standalone)` → `{gps_x, ble_x, tag_x0, tag_x1, max_left_px}` (−1 = not drawn).

| ID | Case | Expected |
|---|---|---|
| UIH-01 | All 8 combinations of (pps, ble, standalone) | no two drawn elements overlap; every element lies within 0..127; a ≥ 2 px gap between neighbours |
| UIH-02 | `ble=false` matches today | `gps_x == 105` (pps) and the tag's right edge == 103 (pps) / 127 (no pps), exactly as the current `draw_mode_tag` |
| UIH-03 | Badge positions | pps+ble → `ble_x == 92`; ble only → `ble_x == 117` |
| UIH-04 | Worst case `max_left_px` | pps+ble, `USB` tag → `max_left_px == 70` |
| UIH-05 | Every mechanical header string fits | for each string `status_text` can return (enumerated from `ui_format.h`) and for `"28,800 bph"`-style rates up to `"432,000 bph"`: `text_w ≤ max_left_px` in the worst case |
| UIH-06 | Shortened strings | `ui_status_text` returns `no stream` / `clock check` / `listening` for the corresponding states (replaces the old expectations at `test_ui_format.cpp:206,214`) |
| UIH-07 | Quartz header | for every quartz tag (`MOTOR`, `LCD`, `FORK`, `QUARTZ`) × calibrated/uncal × worst-case layout: the tag + badge end ≤ `max_left_px`; the badge text is `UNC` only when `UNCAL` wouldn't fit |
| UIH-08 | Rune bitmap | the 9 rows match the documented pattern; the bitmap is 5 bits wide (no bits beyond the 5th) |

### 3.5 CLI retirement (owner A8)

| ID | Case | Expected |
|---|---|---|
| CLI-01 | Size saving | `pico2_pcm1808` (no CLI) vs `pico2_pcm1808_cli`: flash smaller by ≥ 40 KB, RAM smaller by ≥ 1.5 KB; report the exact numbers |
| CLI-02 | Sources really excluded | the `pico2_pcm1808` build log / `.pio/build/pico2_pcm1808/src/` contains no `cli/` objects and no `serial_cli.cpp.o` |
| CLI-03 | CLI builds unchanged | `pico2_pcm1808_cli` on hardware (or the existing Pico 2): CDC port present; `help`, `status`, `diag`, `dsp timing` behave as before |
| CLI-04 | `Serial.` audit | `grep -rnE "\bSerial\." src include` lists only lines inside `#if TG_SERIAL_CLI` blocks or in CLI sources (review gate; the reviewer reads each hit) |
| CLI-05 | Main has no CLI `#if` | `TimegrapherStand_main.cpp` calls `serial_cli_init()` / `serial_cli_tick()` unconditionally; the stubs compile away (check the `nm` of the no-CLI ELF: no `serial_cli_*` symbols) |
| CLI-06 | Parity appendix | `HID_CONTROL_PLANE.md` has a row for every CLI command in the A8 brief's list, each mapped or justified |
| CLI-07 | CDC kept, USB untouched | `git diff` of A8's work doesn't touch `src/usb_audio.cpp`; the no-CLI ELF still contains the CDC driver and the descriptor callback (`nm` shows `tud_cdc_line_state_cb`, the 1200-baud handler of F-18); no `SerialTinyUSB.end` / `clearConfiguration` call anywhere in `src/` |

## 4. Firmware: build and static checks (owners A4, A2, A3)

| ID | Case | Expected |
|---|---|---|
| BLD-01 | Non-W envs | `pio run -e pico2 -e pico2_synthetic -e pico2_pcm1808 -e kiwi -e kiwi_synthetic -e kiwi_pcm1808 -e pico2_pcm1808_cli` succeeds. `pico2_pcm1808_cli` RAM ≤ baseline (164 600 B) + 1 024 B, flash ≤ baseline + 6 KB (framing, arbiter, new ops). `pico2_pcm1808` (no CLI) RAM and flash **below** the baseline |
| BLD-02 | W envs | `pio run -e pico2w -e pico2w_synthetic -e pico2w_pcm1808 -e pico2w_synthetic_noble` succeeds; `pico2w_pcm1808` RAM ≤ 314 572 B (60 %; raised from 50 % on 2026-11-23 for the timing engine, TIMING_ENGINE_PLAN.md), flash ≤ 720 KB |
| BLD-03 | CI config | the YAML parses; the matrix contains `pico2w_pcm1808`, `pico2w_synthetic` |
| BLD-04 | Static analysis | the CI cppcheck command (from `ci.yml`) exits 0 over `src include`; `git clang-format --diff <base>` is empty for changed lines |
| BLD-06 | Core-1 default build | `pio run -e pico2w_synthetic` (core 1 is the default of every W env) succeeds; the link map shows `initVariant` from `variants/rpipico2w_tg`, not from the framework variant. `tests/check_ble_core.sh --build` checks the ELF |
| BLD-05 | Forbidden-API grep (review aid) | `grep -n "setValue" src/ble_control.cpp` shows no call on the TX characteristic; `grep -nE "malloc|new |realloc" src/ble_control.cpp src/control/*.cpp` only in setup-time code, with a comment |

### 4.1 Radio core selection (owner A9)

| ID | Case | Expected |
|---|---|---|
| CORE-01 | Both placements build | `pico2w_synthetic` (core 1, the default) and a temporary core-0 env (stock variant, `-DTG_BLE_CORE=0`) build; `tests/check_ble_core.sh --build` creates the second from a temporary config. In the core-0 ELF `ble_core_setup1` is absent or empty, and in the core-1 ELF `initVariant` doesn't call `init_cyw43_wifi` (`arm-none-eabi-objdump -d`) |
| CORE-02 | No BT lock or BTstack in loop code (static, `tests/check_ble_core.sh`) | No function reachable from `loop()` calls `__lockBluetooth`, `BluetoothLock`, `async_context_acquire_lock_blocking`, `att_server_*`, `gap_*`, `hci_*` or `cyw43_arch_gpio_put` (grep over `src/` with an allow-list of BT-context files and functions, each allow-list entry commented with why). Also: `BLE.startAdvertising` appears exactly once, in `ble_control_begin()` |
| CORE-03 | Core-0 hold (host-testable helper) | The `wait_radio_ready(flag, cap_ms, now_fn)` helper returns true when the flag is set before the cap, false at the cap; it never calls `delay`/`yield` (injected clock only). Tests: flag set at 0 ms, 500 ms, never (returns false at 3000 ms) |
| CORE-04 | Variant drift | `variants/rpipico2w_tg/pins_arduino.h` is byte-identical to the framework's `variants/rpipico2w/pins_arduino.h` for the pinned framework version; `init.cpp` differs only in the `TG_BLE_CORE` branch; the header comment names the source version |
| CORE-05 | Core-1 stack headroom (*hardware*) | After HW-13 on `TG_BLE_CORE=1`: `getLink.ble.core1StackFree ≥ 2048` B; `getLink.ble.core == 1` |

---

## 5. Docs and the example page (owners A5, A6)

| ID | Case | Expected |
|---|---|---|
| DOC-01 | Relative links in the new and changed docs resolve | a small script or manual check: every `](…md)` target exists |
| DOC-02 | `docs/hid/timegrapher_hid.d.ts` type-checks | `npx -y -p typescript@5 tsc --noEmit --strict docs/hid/timegrapher_hid.d.ts` exits 0 |
| DOC-03 | UUIDs consistent | the 4 UUIDs are byte-identical across `BLE_CONTROL_PLANE.md`, `ble_control.cpp`, `timegrapher_hid.d.ts`, `example.html` and the web `constants.ts` (grep) |
| DOC-04 | Spec parity | `BLE_CONTROL_PLANE.md` contains §4.1–§4.7 of the plan (headings present, tables complete) |
| EX-01 | example.html WebHID unchanged | on desktop Chrome with the existing Pico 2: connect, `hello`, `getState`, `subscribe` all work as before |
| EX-02 | example.html BLE connect | with a Pico 2 W: the chooser shows `TG-XXXX`; `hello` shows `link.kind=="ble"` |
| EX-03 | BLE probe report | the probe panel prints a JSON report with every field listed in A6's brief |
| EX-04 | Throughput test | reports ping RTT mean/p95 and per-op notify rates over 10 s |

---

## 6. Web: shared framed transport (`src/transport/FramedTransport.test.ts`), owner W1

Use a minimal `TestTransport extends FramedTransport` whose `sendFrame` records frames and lets the test inject reply frames with `onFrame()`. Use fake timers.

| ID | Case | Expected |
|---|---|---|
| FT-01 | `request` before `open` | rejects `notOpen` |
| FT-02 | Single in-flight | two requests issued together | the second's frames are sent only after the first resolves |
| FT-03 | Reply matching by id | a reply with another id | ignored; the request still pending |
| FT-04 | Error reply | `{ok:false,error:{code:"badParam",message:"x"}}` | `DeviceError` with code `badParam` and the op name |
| FT-05 | Timeout | no reply by the timeout | `timeout`; the next queued request is sent |
| FT-06 | Late reply after timeout | delivered after FT-05 | ignored, no throw |
| FT-07 | Notify dispatch | a notify frame | every listener called; a listener throwing doesn't stop the others |
| FT-08 | Oversize request | a > 2030-byte body | rejects `protocol`; nothing sent |
| FT-09 | Link lost | `onLinkLost('x')` with 1 in flight + 2 queued | all 3 reject `disconnected`; disconnect listeners fire once with the reason |
| FT-10 | `close()` | idempotent; rejects pending with `disconnected`; disconnect listeners **not** fired |
| FT-11 | Keepalive | open; no requests for 3000 ms (fake timers) | exactly one `ping` sent; another after 3000 ms more of idleness; a real request resets the idle clock (no ping within 3000 ms after it) |
| FT-12 | Keepalive failure is harmless | the keepalive `ping` times out | no listener fired, no throw; the next user request works |
| FT-13 | `controlReleased` | a notify `{op:"controlReleased",data:{to:"ble"}}` arrives | pending requests reject `disconnected`; disconnect listeners fire once with `"Control taken over by ble"`; the transport is closed |
| — | Regression | the existing `WebHidTransport.test.ts` passes **unmodified** | green |

---

## 7. Web: `WebBleTransport` (`src/transport/WebBleTransport.test.ts`), owner W2

All tests use the fake Bluetooth stack from W2 (no real `navigator.bluetooth`).

| ID | Case | Expected |
|---|---|---|
| WB-01 | `isWebBleSupported()` | false without `navigator.bluetooth`, true with it |
| WB-02 | `requestDevice` options | `filters` = `[{services:[SERVICE]},{namePrefix:'TG-'}]`, `optionalServices` includes SERVICE |
| WB-03 | Open sequence | `gatt.connect` → `getPrimaryService(SERVICE)` → RX/TX/INFO lookups → INFO read → `startNotifications` on TX; **notifications started before the first RX write** (order asserted) |
| WB-04 | Chooser cancelled | `requestDevice` rejects `NotFoundError` | `DeviceError('notOpen','No device selected')` (exact message) |
| WB-05 | MTU too small | INFO `mtu:23, seg:0` on all reads | 3 re-reads 300 ms apart (fake timers), then `linkUnsupported`; `gatt.disconnect` called |
| WB-06 | MTU becomes OK on retry | INFO 23, then 185 | open succeeds after one retry |
| WB-07 | Request/response | `ping` | RX gets one unpadded frame (`5 + payloadLen` bytes), the reply notification resolves with the result |
| WB-08 | Sequential writes | a 5-fragment request; the fake write takes 20 ms | writes never overlap (the fake throws `NetworkError: GATT operation already in progress` on overlap, as Chrome does); all 5 in order |
| WB-09 | Write mode | default uses `writeValueWithResponse`; with `writeMode:'withoutResponse'` it uses `writeValueWithoutResponse` | correct method called |
| WB-10 | Notifies | an unsolicited notify frame | `onNotify` listeners called with the parsed object |
| WB-11 | Unexpected disconnect | the fake fires `gattserverdisconnected` with 1 pending request | rejects `disconnected`; disconnect listeners once; later `request` → `notOpen` |
| WB-12 | `close()` | idempotent; calls `stopNotifications` (errors swallowed), removes both listeners (the fake counts them), `gatt.disconnect()`; disconnect listeners **not** fired |
| WB-13 | Write failure | the fake write rejects `NetworkError` | the request rejects `disconnected` |
| WB-14 | Oversized device value | a 64-byte notification | dropped; no throw; later valid frames still work |
| WB-15 | Identity | `kind === 'webble'`; `productName === device.name` | — |
| WB-16 | Reconnect with a new instance | after close, a new `WebBleTransport({device})` opens the same fake device | works; the old instance's listeners aren't invoked (counted) |
| WB-17 | `getAvailability` | false → `isWebBleAvailable()` resolves false; missing API → resolves true | — |
| WB-18 | Large reply | a 2047-byte reply (36 notifications) | reassembled; resolves |
| WB-19 | Reassembly timeout | 1 of 2 reply fragments, then 600 ms of silence | the request times out normally (no partial delivered) |
| WB-20 | Segmented mode (*conditional*, §4.5) | INFO `seg:1, mtu:23` | the open succeeds; writes are ≤ 20-byte values with a START header; replies reassemble |
| WB-21 | `linkBusy` propagates | a device `{ok:false,error:{code:"linkBusy"}}` reply | rejects `DeviceError('linkBusy', <device message>)` (not `internal`) |
| WB-22 | Write-method fallback (R-4) | the RX characteristic lacks the configured write method (each of the 4 combinations) | the other method, then legacy `writeValue`, is used; none at all → `open()` rejects `linkUnsupported` |
| WB-23 | Chooser filter retry (R-4) | the first `requestDevice` rejects with `TypeError` | one retry with the service filter only; a second failure propagates; `NotFoundError` is not retried |

### 7.1 Contract suite over BLE (`tests/contract/contract.ble.test.ts`), owner W2

| ID | Case | Expected |
|---|---|---|
| CT-BLE-01 | `runTransportContract(createBleSimHarness)` | **every** test in `transportContract.ts` passes (same expectations as the simulator harness) with `WebBleTransport` ↔ `FakeGattServer` ↔ `VirtualDevice`, MTU 185, 7.5 ms write latency |
| CT-BLE-02 | The same suite at MTU 66 (the edge) | passes |
| CT-BLE-03 | The harness has a "drop link" hook (to ensure disconnect paths run under the contract timing) | one added contract-adjacent test: drop mid-`getState` → `disconnected`, and the harness disposes cleanly |

---

## 8. Web: store, subscriptions, UI, platform (owner W3)

| ID | Case | Expected |
|---|---|---|
| CS-01 | Factory | `defaultTransportFactory('webble')` instanceof `WebBleTransport` |
| CS-02 | `webBleAvailable` | false without the API; follows `getAvailability()`; updates on `availabilitychanged` |
| CS-03 | Connect over BLE (injected fake factory) | status `connected`, `kind 'webble'`, hello stored |
| CS-04 | Cancelled chooser | no snackbar; status back to `disconnected` |
| CS-05 | `linkUnsupported` | an error snackbar whose text mentions the MTU and suggests another browser/device |
| CS-06 | Label | `deviceLabel` ends with ` · BLE` for `webble`, unchanged for `webhid` |
| CS-07 | Disconnect event | warning snackbar, state reset (as for WebHID) |
| CS-08 | `hello.link` mismatch tolerance | a hello without `link` (old firmware over HID) still connects |
| OWN-01 | Free device | the simulator with no foreign owner; connect | no dialog; connected |
| OWN-02 | Held by another link | `setForeignOwner('usbHid')`; connect over `webble` | after `hello` the store pauses in a `needsTakeover` state; the dialog shows "USB"; no other request has been sent |
| OWN-03 | Take over | OWN-02, then Take over | `claim {takeover:true}` sent, then the normal connect continues; status `connected` |
| OWN-04 | Cancel | OWN-02, then Cancel | transport closed; status `disconnected`; no snackbar error |
| OWN-05 | `linkBusy` later | connected; the simulator switches to a foreign owner; the next request fails `linkBusy` | the dialog appears; Take over → re-claim, and the failed view refreshes |
| OWN-06 | `controlReleased` | connected; the simulator emits `controlReleased {to:'usbHid'}` | warning snackbar "Another app took control over USB"; store reset |
| OWN-07 | Old firmware | a hello without `control` | connects with no dialog (back-compat) |
| SUB-01 | Rate clamp on BLE | requesting metrics 20 / beat 5 / timing 5 / quartz 5 on `webble` → the `subscribe` request carries 10 / 2 / 2 / 2 |
| SUB-02 | No clamp on WebHID | the same request on `webhid` is unchanged |
| SUB-03 | Single source | the caps come from the exported `LINK_RATE_CAPS` (the test imports it, so a retune doesn't break tests) |
| PL-01 | `isIos` on iPhone UA | true |
| PL-02 | iPadOS desktop UA (`MacIntel`, `maxTouchPoints 5`) | true |
| PL-03 | Mac desktop (`MacIntel`, `maxTouchPoints 0`) | false |
| PL-04 | `isLikelyBluefy` | true when iOS && `'bluetooth' in navigator`; false otherwise |
| PL-05 | Pure functions | the helpers take an injected `{userAgent, platform, maxTouchPoints, hasBluetooth}`; no global access in the tests |
| UI-01 | ConnectPrompt with BLE available | the "Real device (Bluetooth)" button is visible and calls `connect('webble')` |
| UI-02 | ConnectPrompt without BLE or HID on iOS | guidance text mentions Bluefy; "Copy link" copies `location.href` (clipboard mocked) |
| UI-03 | ConnectPrompt in a Bluefy-like env | BLE and Simulator shown; the WebHID button and its hint hidden |
| UI-04 | Desktop Chrome (both APIs) | WebHID, BLE and Simulator buttons all shown |
| UI-05 | AppBarConnection menu | includes the BLE item, enabled iff `webBleAvailable` |
| UI-06 | Accessibility | the new buttons have accessible names; the guidance is in an `aria-live` region or an alert role |

### 8.1 Web docs (owner W4)

| ID | Case | Expected |
|---|---|---|
| WDOC-01 | README BLE section | covers desktop, Android and iPad/Bluefy steps, `TG-XXXX`, the limits (foreground, one BLE host, no pairing) |
| WDOC-02 | Type sync | `npm run sync-types` produces no diff after the run (idempotent); `npm run typecheck` green |

---

## 9. Hardware tests (2026-11-23), owners: manager + user

Record every result in `docs/acceptance_results.md` ("BLE (Pico 2 W)") with the firmware `git describe`, env, client device/OS/browser versions, measured numbers and pass/fail.

**Diagnostics source:** G0/G1 builds include the CLI (`pico2w_synthetic_noble`, `pico2w_synthetic_cli`), so `diag`, `dsp timing` and `ble` work there. G2 builds have no CLI, so use `getDiag`, `getTiming` and `getLink` from `example.html`'s raw request box or the web app's raw console.

| ID | Gate | Setup | Procedure | Pass criteria |
|---|---|---|---|---|
| HW-00 | Baseline | the existing **Pico 2**, current committed firmware, `pico2_synthetic` (it still has the CLI) | Record 10 min of USB audio; `host_analyze.py`; `diag`; `dsp timing` | Record the `diag` counters, loop busy % and max loop µs. Every W comparison is against these numbers |

| HW-01 | G0 | `pico2w_synthetic_noble`, then `pico2w_pcm1808` (PCM1808 wired) | Flash; plug into the Mac; check enumeration (UAC2 mic, CDC, HID); CLI `version`, `status`, `diag`; open the web app over WebHID: hello, Timegrapher view | Enumerates within 3 s of plug-in (also with BLE builds; note the CYW43 init time from the boot log); the LED blinks via CYW43; `hello.board=="Pico2W"`; `pico2w_pcm1808` captures audio (the PIO claim works, R-9) |
| HW-02 | G0 | `pico2w_synthetic_noble` vs `pico2w_synthetic_cli` (advertising, not connected) | Record 10 min of USB audio each (`host_capture.md`); run `host_analyze.py`; read `diag` and `dsp timing` | No dropouts in either; `pcm_ring_underrun_count == 0`, `pcm_ring_overrun_count == 0`, `dma_overrun_count == 0`; loop busy % ≤ baseline + 5 points; max loop µs within the existing budget |
| HW-03 | G1 | BLE build (echo) | nRF Connect: scan | `TG-XXXX` visible, connectable, service UUID `c137e7c0-…` in the advertisement; the name matches `ble` CLI output; advertisement ≤ 31 bytes |
| HW-04 | G1 | BLE build | Connect from each client; read INFO (nRF Connect / probe); `ble` CLI | Record the MTU per client. **Must:** iPad + Bluefy ≥ 66, macOS Chrome ≥ 66. **Should:** Windows Chrome, Android Chrome ≥ 66. Must failing → §4.5 |
| HW-05 | G1 | Echo build, nRF Connect | Enable notify on TX; write hex `000100011A7B2276223A312C226964223A312C226F70223A2270696E67227D` (the `ping` frame, 31 B) to RX | The same 31 bytes come back as a notification (echo). With the M2 build the same write returns a `ping` reply frame whose JSON has `"id":1,"ok":true`. A second frame for the M2 test: hello = `000200011B7B2276223A312C226964223A322C226F70223A2268656C6C6F227D` |
| HW-06 | G1/G2 | Synthetic BLE build, USB audio recording running | Echo flood (G1: the probe sends frames back-to-back for 10 min) / G2: the BLE session subscribed at **max** rates for 10 min | Same audio pass criteria as HW-02; `ble` `rx_dropped == 0`; no disconnects |
| HW-07 | G2 | M2 build, probe throughput test from iPad/Bluefy and desktop Chrome | ping ×100; subscribe max rates for 10 s | Record the RTT mean/p95 and the delivered Hz per op. Set `LINK_RATE_CAPS.webble` to ≤ 80 % of the delivered maxima on the slowest must-have client. Pick the write mode (with/without response) from the RTT |
| HW-08 | G2 | Web app (GitHub Pages or `vite preview` over HTTPS) in Bluefy on iPad | Connect (BLE), then walk every view: Timegrapher, quartz modes, settings, timebase calibration (incl. `saveTimebase`), GPS, diagnostics, raw console | Every view works; no console errors; the label shows `· BLE`; disconnect/reconnect from the UI works |
| HW-09 | G2 | Web app in desktop Chrome over BLE | The HW-08 checklist, abbreviated (connect, Timegrapher 5 min, settings patch) | Pass |
| HW-10 | G2 | Desktop Chrome (web app, USB HID) and iPad Bluefy (web app, BLE) | (1) Desktop connects and subscribes. (2) iPad connects: the takeover dialog says "USB"; Cancel → the iPad disconnects and the desktop is unaffected. (3) iPad again → Take over: the iPad works; the desktop shows "Another app took control over Bluetooth" within 1 s and resets. (4) Close the iPad tab: after the BLE disconnect the desktop reconnects **without** a dialog. (5) Close the desktop tab with no BLE host, wait 11 s, connect the iPad: no dialog (the lease expired). (6) The `example.html` raw `getState` from the non-owner returns `linkBusy` | Every step as described; no stuck ownership; no pushes reach the non-owner |
| HW-11 | G2 | iPad/Bluefy | 20 cycles of: close the tab / background Bluefy / toggle iPad Bluetooth / walk out of range | The device resumes advertising within 2 s every time; reconnect succeeds; `ble` `disconnects` counts match; the BLE subscription is cleared on disconnect (no pushes after reconnect until re-subscribed) |
| HW-12 | G2 | Power bank, no USB host; device mode STANDALONE | Boot; connect over BLE; run a measurement | BLE works without USB; measurement data flows |
| HW-13 | G2 | Soak: USB audio streaming + BLE session subscribed at `LINK_RATE_CAPS` | 1 h | Zero underruns/overruns; zero BLE disconnects; `ble` counters sane; free RAM stable (no growth) |
| HW-14 | G2 | BLE connected, subscribed | 10 × `saveTimebase` and timebase-log writes (per `TIMEBASE_CALIBRATION.md`) | No BLE disconnect; no fault; replies arrive |
| HW-15 | G2 (*optional*) | iPad at 1 m, 5 m, 10 m line of sight | Record RSSI (nRF Connect) and the ping RTT | Informational |
| HW-16 | G1 | example.html BLE probe in Bluefy, desktop Chrome, Android Chrome | Run the probe | Record the feature matrix; any missing feature → R-4 mitigation tasks for W2 |
| HW-17 | G2 | `pico2w_pcm1808` with the OLED, PPS build | Observe the header: power on (advertising), connect the iPad, disconnect, `ble off` (CLI build), in mechanical and quartz modes, and in USB and SA modes | Badge hollow while advertising, inverted within 1 s of connect, hollow within 2 s of disconnect, absent when BLE is off and on the non-W board; no overlap with the tag, GPS or header text in any mode; the rune is legible (refine the bitmap if not, and update UIH-08) |
| HW-18 | G0 | `pico2w_synthetic` (no CLI), running | List the USB interfaces (macOS: `ioreg -p IOUSB -l -w0` / System Information); open the serial port in a terminal for 30 s while USB audio records; then, with the device still running, `pio run -e pico2w_synthetic -t upload` without touching BOOTSEL | The CDC port, UAC2 mic and HID are present. The terminal shows at most the UUID line from boot and nothing after, and the open terminal causes no audio dropouts. The upload auto-resets into BOOTSEL and succeeds; the device comes back and WebHID connects |
| HW-19 | G2 | core 0 (temporary env of CORE-01) vs `pico2w_synthetic` (core 1); `pico2w_synthetic_noble` as the reference | For each: 10 min of USB audio with the BLE session subscribed at `LINK_RATE_CAPS` from the iPad (Bluefy), plus one connect/disconnect cycle every minute (scripted in the probe or by hand); read `getTiming` (`loopUsMax`, `loopBusyPct`, `dspBlockUsMax`, `underruns`) and `getDiag` every minute; read `getLink.ble.core` | Each build: zero underruns/overruns and no audio dropouts (`host_analyze.py`). Against noble: `loopUsMax` increase ≤ 500 µs and `loopBusyPct` increase ≤ 3 points. Record all three builds' numbers. **Decision (§5.8):** core 1 if it passes (then CORE-05 and HW-13 on core 1); otherwise core 0 if it passes; otherwise the R-1 ladder. **Decided 2026-11-23: core 1 is the default** |

---

## 10. Traceability summary

| Requirement (plan) | Tests |
|---|---|
| §4.3 framing unchanged, unpadded on BLE | FR-RX-*, FR-TX-08, FR-RT-*, SES-12, WB-07, CT-BLE-01 |
| F-15 bug fix | FR-TX-07, SES-05 |
| §4.4 connection behaviour | PACE-06, SES-08/09, WB-11, HW-11 |
| §4.6 hello additions | HJ-01..03, CS-08 |
| §4.7 exclusive control (lease, takeover) | ARB-01..16, HJ-04, FT-11..13, OWN-01..07, HW-10 |
| §5.6 OLED Bluetooth badge | UIH-01..08, HW-17 |
| §5.7 CLI retirement (CDC kept) | CLI-01..07, BLD-01, HW-18 |
| §4.6 new ops (`getLink`, `getDiag`) | HJ-05, HJ-06 |
| §5.8 radio core selection | CORE-01..05, BLD-06, HW-19 |
| §5.4 BT-context safety | RING-08, PACE-*, BLD-05, CORE-02, HW-06, HW-13 |
| §6.3 rate caps | SUB-01..03, HW-07 |
| §6.4 iOS guidance | PL-*, UI-02/03, HW-08 |
| USB audio not degraded | HW-02, HW-06, HW-13, HW-19 |
| Non-W builds unchanged | BLD-01, FT regression, SES-*, EX-01, HW-01 |

---

## 11. Detection pre-filter, matched filter, synthetic impairments (revision 4)

Plan §13. Host tests in `tests/test_detect_filter.cpp` (DPF, MF) and `tests/test_synth_impair.cpp` (SYN), added to `tests/run_all.sh`. Hardware rows use the synthetic source, so no PCM1808 is needed.

| ID | Test | Pass |
|---|---|---|
| DPF-01 | `detectFilter` disabled | Event and beat output for a recorded tick sequence is bit-identical to the build without the block |
| DPF-02 | FIR design | 65 symmetric taps; group delay exactly 32 samples; for each preset, gain at the band centre within 1 dB of 0 dB and ≥ 30 dB down at 50 Hz (wide) / the stop-band edge |
| DPF-03 | Notch | `notchHz` 50 / 60: ≥ 30 dB attenuation at the notch frequency, ≤ 0.5 dB change at 1 kHz–8 kHz |
| DPF-04 | Delay compensation | A synthetic impulse at sample index *k* gives an event `onset` within ±1 sample of *k*, with the FIR on and off |
| DPF-05 | Warm-up | No event opens in the first 64 samples after a stream reset with the FIR on |
| DPF-06 | Toggle | Enabling, disabling or changing a parameter mid-stream resets filter state and any open event, keeps the beat history, and never produces an event spanning the change |
| DPF-07 | Rate invariance | The tick-tock loop with hum (`humAmp` 0.05) and noise (`noiseAmp` 0.01): with `detectFilter` on (notch 60, wide), rate within ±0.1 s/d of the clean-loop rate and beat error within ±0.05 ms |
| DPF-08 | *Hardware*, `pico2w_synthetic`, impairments on | With 60 Hz hum + noise, record lock / rate / amplitude / beat error with `detectFilter` off and on (notch 60, wide). Pass: on locks and reads the clean values (DPF-07 tolerances); amplitude within ±5° of the clean loop. Record how "off" behaves (informational) |
| DPF-09 | *Hardware*, cost | `getTiming` with `detectFilter` on: `dspBlockUsMax5s` < 2500 µs (half the 5 ms budget) over 10 min; `dspUsPerS` increase recorded |
| DPF-10 | *Hardware with a PCM1808* (later) | Real pickup, bench noise: same comparison as DPF-08; decides the §13.9 default |
| MF-01 | Normalized score | The same tick at 0.01, 0.1 and 0.5 FS scores within ±0.02 of each other; white noise scores < 0.3 |
| MF-02 | Quiet pickup | Learned template, ticks at 0.02 FS: no event is vetoed (regression for §13.4 defect 1) |
| MF-03 | Per-sample veto | A click 2 ms before a tick in the same 5 ms block: the click is vetoed, the tick is detected |
| MF-04 | Learning source | Off-grid clicks (`clickRate` 5/s) never change the template |
| MF-05 | Convergence stop | Learning stops after 20 consecutive updates with correlation > 0.98; a reset re-enables it |
| SYN-01 | Defaults | All impairment parameters 0: the generated samples are identical to the clean loop |
| SYN-02 | Reproducible noise | Same seed → same sequence after a stream reset |
| SYN-03 | Op gating | `setSynth` / `getSynth` answer only when `caps.synth`; other builds reply `badRequest` |
| SYN-04 | *Hardware*, `pico2w_synthetic` | `setSynth` over BLE and over USB HID changes what `getMetrics` reports (pre RMS rises with `noiseAmp`) and what the USB recording contains |

---

## 12. BLE throughput: packed beats, large frames, connection interval (revision 5)

Plan §14. Host tests in `tests/test_beats_packed.cpp` (BP) and `tests/test_control_session.cpp` / `tests/test_frame_ring.cpp` (LF).

| ID | Test | Pass |
|---|---|---|
| BP-01 | Round trip | Encode a batch of 1, 8 and 32 beats (accepted, started, rejected; both parities; amplitude valid/invalid), decode with a reference decoder in the test: every field equal within its quantum (u ±0.05 µs, amp ±0.005°, lift ±0.5 µs, frac ±1/256 sample) |
| BP-02 | Size | 8 beats = 162 bytes binary, 216 base64 characters |
| BP-03 | Batch breaks | A grid change, a `dSeq` of 256, and a `dK` outside i16 each end the batch at that beat; the next batch starts with it |
| BP-04 | Fit | A batch never makes the envelope exceed 2048 bytes; the `emitted` count says how many beats went |
| BP-05 | Subscribe | `beatsPacked: true` sends `beatsPacked` notifies and no `beats`; `false` restores `beats` if that is still subscribed; `getState`/subscription echo shows the field |
| BP-06 | Caps / history | `hello.caps.beatsPacked == 1`; `getBeats {packed: true}` returns `{p}` in seq order |
| LF-01 | Ring slot size | `FrameRing<16, 255>` (template argument = largest frame; slots are 256 B) stores and returns 255-byte frames; existing RING tests pass for the default `FrameRing<N>` |
| LF-02 | Opt-in | BLE-like link (`max_tx_frame` 255): before `hello {frameMax}` frames are ≤ 63; the `hello` reply itself is ≤ 63; later replies use 255-byte frames (chunk 250) |
| LF-03 | Bounds | `frameMax` below 63 is ignored (63 kept); 1000 → 255; with the link's `max_tx_frame` at 120, 255 → 120 |
| LF-04 | Reset | After `on_link_down()` the frame size is 63 again |
| LF-05 | HID unaffected | HID-like link (`max_tx_frame` 63, padded): `frameMax` has no effect; frames stay 63 padded |
| HW-07b | *Hardware*, macOS + bleak (and iPad/Bluefy later) | Repeat HW-07 with large frames on and `beatsPacked`: record notifications/s, bytes/s, ping RTT, `connIntervalUs`, `connParamStep`. Set `LINK_RATE_CAPS.webble` from it |
| HW-20 | *Hardware*, 36,000 bph | `TG_SYNTH_SPEED=2` build (loop at double speed: 36,000 bph); BLE subscribed with `beatsPacked`, `beat` 2 Hz, `metrics` 5 Hz for 10 min while USB audio records. Pass: every beat seq delivered (no gaps in seq), notify lag p95 < 300 ms (beat `tS` vs arrival), audio sample check clean, link usage < 40 % of HW-07b capacity |
