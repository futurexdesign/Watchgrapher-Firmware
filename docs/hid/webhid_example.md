# WebHID example (Timegrapher control)

Minimal browser-side sketch for the HID control plane. Normative details: [`../HID_CONTROL_PLANE.md`](../HID_CONTROL_PLANE.md). Types: [`timegrapher_hid.d.ts`](timegrapher_hid.d.ts).

**Requires:** Chromium-based browser, HTTPS or `localhost`, user gesture for `requestDevice`. Firmware with `CFG_TUD_HID=1` (current builds). Interactive page: [`example.html`](example.html) — full control UI (block enable/params, bypass, metrics/timing/beat/spectrum/matched, events/errors, subscribe pushes, presets, timebase calibration, quartz timing, device mode / run control, beats, GPS detail and the timebase log).

---

## 1. Connect

```js
const USAGE_PAGE = 0xffa0;
const USAGE = 0x0001;
const REPORT_ID = 0x01;

async function connectTimegrapher() {
  const devices = await navigator.hid.requestDevice({
    filters: [{ usagePage: USAGE_PAGE, usage: USAGE }],
  });
  const device = devices[0];
  if (!device) throw new Error('no device selected');
  if (!device.opened) await device.open();
  return device;
}
```

Optional secondary filter once VID/PID are fixed in your build: `{ vendorId: 0xcafe, productId: 0x4003 }` (current `usb_audio.cpp` IDs).

---

## 2. Framer (63-byte WebHID buffers)

Chromium delivers / accepts report payloads **without** the report ID byte. Layout:

| Offset | Field |
|---:|---|
| 0 | flags (bit0 = moreFragments) |
| 1 | msgSeq |
| 2 | fragIndex |
| 3 | fragCount |
| 4 | payloadLen |
| 5.. | UTF-8 chunk (≤ 58 bytes) |

```js
const FLAG_MORE = 0x01;
const MAX_CHUNK = 58;
const MAX_JSON = 2030; // device accepts requests of <= 35 fragments (2030 bytes); replies reach 2047
const REPORT_BYTES = 63;

class HidFramer {
  constructor() {
    this._txSeq = 0;
    this._rx = null; // { seq, parts, count, timer }
    this.onMessage = null; // (obj) => void
  }

  /** Build OUTPUT reports for one JSON object. */
  encode(obj) {
    const text = new TextEncoder().encode(JSON.stringify(obj));
    if (text.length > MAX_JSON) throw new Error('json too large');
    const fragCount = Math.max(1, Math.ceil(text.length / MAX_CHUNK));
    const seq = (this._txSeq++) & 0xff;
    const reports = [];
    for (let i = 0; i < fragCount; i++) {
      const start = i * MAX_CHUNK;
      const chunk = text.subarray(start, start + MAX_CHUNK);
      const buf = new Uint8Array(REPORT_BYTES); // always a full, zero-padded report
      const more = i + 1 < fragCount ? FLAG_MORE : 0;
      buf[0] = more;
      buf[1] = seq;
      buf[2] = i;
      buf[3] = fragCount;
      buf[4] = chunk.length;
      buf.set(chunk, 5);
      reports.push(buf);
    }
    return reports;
  }

  /** Feed one INPUT report payload (DataView or Uint8Array). */
  push(data) {
    const u8 = data instanceof Uint8Array ? data : new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
    if (u8.length < 5) return;
    const flags = u8[0];
    const seq = u8[1];
    const fragIndex = u8[2];
    const fragCount = u8[3];
    const payloadLen = u8[4];
    const chunk = u8.subarray(5, 5 + payloadLen);

    if (!this._rx || this._rx.seq !== seq) {
      this._rx = { seq, parts: new Array(fragCount), count: fragCount, got: 0 };
    }
    if (fragIndex >= this._rx.count || this._rx.parts[fragIndex]) return;
    this._rx.parts[fragIndex] = chunk;
    this._rx.got++;

    const done = this._rx.got === this._rx.count && (flags & FLAG_MORE) === 0
      ? true
      : this._rx.got === this._rx.count;
    if (!done) return;

    let total = 0;
    for (const p of this._rx.parts) total += p.byteLength;
    const out = new Uint8Array(total);
    let o = 0;
    for (const p of this._rx.parts) {
      out.set(p, o);
      o += p.byteLength;
    }
    this._rx = null;
    const text = new TextDecoder().decode(out);
    const obj = JSON.parse(text);
    if (this.onMessage) this.onMessage(obj);
  }
}
```

---

## 3. Request / response helper

```js
class TimegrapherHidClient {
  constructor(device) {
    this.device = device;
    this.framer = new HidFramer();
    this._pending = new Map(); // id -> { resolve, reject }
    this._nextId = 1;
    this.onNotify = null; // (msg) => void

    this.framer.onMessage = (msg) => {
      if (msg.notify) {
        if (this.onNotify) this.onNotify(msg);
        return;
      }
      const p = this._pending.get(msg.id);
      if (!p) return;
      this._pending.delete(msg.id);
      if (msg.ok) p.resolve(msg.result);
      else p.reject(Object.assign(new Error(msg.error?.message || 'hid error'), { code: msg.error?.code }));
    };

    device.addEventListener('inputreport', (ev) => {
      if (ev.reportId !== REPORT_ID) return;
      this.framer.push(ev.data);
    });
  }

  async request(op, extra = {}) {
    const id = this._nextId++;
    const body = { v: 1, id, op, ...extra };
    const reports = this.framer.encode(body);
    const result = new Promise((resolve, reject) => {
      this._pending.set(id, { resolve, reject });
      setTimeout(() => {
        if (this._pending.has(id)) {
          this._pending.delete(id);
          reject(new Error('hid timeout'));
        }
      }, 2000);
    });
    for (const r of reports) {
      await this.device.sendReport(REPORT_ID, r);
    }
    return result;
  }

  hello() { return this.request('hello'); }
  getState() { return this.request('getState'); }
  patchState(patch) { return this.request('patchState', { patch }); }
  getMetrics() { return this.request('getMetrics'); }
  getBeat() { return this.request('getBeat'); }
  // limit <= 5 always fits one reply; a larger limit can fail with 'tooLarge'
  getEvents(limit = 5) { return this.request('getEvents', { limit }); }
  subscribe(opts) { return this.request('subscribe', opts); }
  getQuartz() { return this.request('getQuartz'); }
  getQuartzPulses(max = 8) { return this.request('getQuartzPulses', { max }); }
  quartzRestart() { return this.request('quartzRestart'); }
  getDevice() { return this.request('getDevice'); }
  setDevice(mode) { return this.request('setDevice', { mode }); } // 'usbMic' | 'standalone'
  start() { return this.request('start'); }
  stop() { return this.request('stop'); }
  getBeats(max = 8) { return this.request('getBeats', { max }); }
  getGps() { return this.request('getGps'); }
  getTimebaseLog(offset = 0) { return this.request('getTimebaseLog', { offset }); }
  clearTimebaseLog() { return this.request('clearTimebaseLog'); }
  setTimebaseLogDelta(logDeltaPpm) { return this.request('setTimebase', { logDeltaPpm }); }
}
```

---

## 4. Typical UI flow

```js
const device = await connectTimegrapher();
const client = new TimegrapherHidClient(device);

client.onNotify = (msg) => {
  if (msg.op === 'metrics') console.log('metrics', msg.data);
  if (msg.op === 'event') console.log('events', msg.data.events);
  if (msg.op === 'error') console.warn('fault', msg.data);
};

const info = await client.hello();
console.log(info.fwVersion, info.caps.blocks);

let state = await client.getState();
console.log('bypass', state.bypass);

state = await client.patchState({
  bypass: false,
  blocks: {
    gain: { enabled: true, params: { db: 6 } },
    notch: { enabled: true, params: { hz: 60 } },
  },
});
console.log('after patch', state.blocks.gain);

await client.subscribe({ metricsHz: 10, beatHz: 2, events: true, errors: true });
```

---

## 5. Quartz watch timing

Quartz timing (stepper motor, LCD, tuning fork) exists when `hello` reports `caps.quartz: true` (PCM1808 builds). Settings are part of the state; the measurement has its own ops. Protocol: [`HID_CONTROL_PLANE.md` §5.3, §8.5, §8.6](../HID_CONTROL_PLANE.md).

Select motor timing with a 60 s gate:

```json
{ "v": 1, "id": 21, "op": "patchState", "patch": { "quartz": { "mode": "motor", "gateS": 60 } } }
```

The reply is the full state; its `quartz` object is the new settings:

```json
{ "v": 1, "id": 21, "ok": true, "result": {
  "bypass": true, "blocks": { "...": {} },
  "quartz": { "mode": "motor", "gateS": 60, "forkHz": 360, "tsMethod": "threshold", "kSigma": 8 } } }
```

Read the measurement:

```json
{ "v": 1, "id": 22, "op": "getQuartz" }
```

```json
{ "v": 1, "id": 22, "ok": true, "result": {
  "mode": "motor", "state": "measuring", "measurementId": 3, "reason": "", "sourceHz": 48000, "timebaseOk": true,
  "elapsedS": 23.4, "gateS": 60, "forkHz": 360, "tsMethod": "threshold", "kSigma": 8,
  "sampleLossRestarts": 0, "rateSDay": 0.9022, "rateValid": true,
  "calPpm": 0, "calSource": "none", "calibrated": false,
  "motor": {
    "valid": true, "pulses": 23, "fitPulses": 23, "stepS": 1, "spanS": 22,
    "rateWithSDay": 0.9022, "rateWithUncalSDay": 0.9022, "rateWithoutSDay": 0.9022, "rateWithoutUncalSDay": 0.9022,
    "inhibition": "may_include_inhibition", "inhibitionPeriodValid": false, "inhibitionPeriodS": 0,
    "events": 0, "cyclesUsed": 0, "jumpLastUs": 0, "jumpLastN": 0, "jumpMeanUs": 0, "jumpMeanN": 0,
    "residualRmsUs": 0.25, "polarityOffsetUs": 0.03, "clipped": false, "clippedPulses": 0, "extraPulses": 0,
    "noiseSigma": 9900, "threshold": 79200, "peakSnr": 51.5, "hintWeakSignal": false, "hintClipped": false,
    "noJumpSpanS": 120, "jumpThresholdUs": 20, "rebuilding": false },
  "lcd": null, "fork": null } }
```

How to present it:

- `rateSDay` is s/d, **positive = gaining**. Show the sign.
- `calibrated: false` → every rate is against the uncorrected board clock (up to ±1.7 s/d off): flag it **UNCALIBRATED** and point to the timebase calibration (`setTimebase`, see `example.html`).
- Always show `motor.inhibition` (handoff §5). Here `may_include_inhibition` means the gate has not yet seen an inhibition cycle, so `rateSDay` is the raw oscillator rate; offer a longer gate (`gateS: 60` or `0` = until two events). With `corrected`, `rateWithSDay` is the headline and `rateWithoutSDay` (raw), `inhibitionPeriodS` and `jumpLastUs` / `jumpLastN` (n at 8192 Hz) are details.
- `hintWeakSignal` → ask the user to move the watch closer to the coil; `hintClipped` → open JP1.

Live updates, without replaying history:

```js
await client.subscribe({ quartzHz: 1, quartzPulses: true });
client.onNotify = (msg) => {
  if (msg.op === 'quartz') showQuartz(msg.data);          // DspQuartz, 1 Hz
  if (msg.op === 'quartzPulse') addPulseRow(msg.data);    // one DspQuartzPulse per motor step
};
```

A change of `measurementId` means a new measurement (restart, mode or setting change, stream reset): clear any pulse list or plot, and key merged pulses by (`measurementId`, `sequence`). When `state` becomes `done` (the motor gate was reached) the result is frozen; `client.quartzRestart()` measures again. `getQuartzPulses` returns at most 8 pulses (`caps.quartzPulsesMax`); the `quartzPulse` notify delivers every pulse.

---

## 6. Device mode, beats, GPS and the timebase log (H6)

Feature presence is advertised in `hello.caps` (`deviceMode`, `beats`, `beatsMax`, `gps`, `pps`, `timebaseLog`, `timebaseLogMax`, `pipeline`); an absent key means the firmware predates it. Protocol: [`HID_CONTROL_PLANE.md` §5.4 to §5.9, §8.7](../HID_CONTROL_PLANE.md).

```js
const info = await client.hello();
if (info.caps.deviceMode) {
  let dev = await client.getDevice();           // { mode, running, runRequested, usbStreaming, streamState, saved }
  if (dev.mode !== 'standalone') dev = await client.setDevice('standalone'); // flash write; `busy` while a USB stream is open
  dev = await client.start();                   // set the run latch (standalone sets it by itself at boot)
}

// Per-beat records for a tick-tock chart. Newest first; seq restarts at 0 on a stream reset.
const { beats } = await client.getBeats(8);
await client.subscribe({ beats: true });         // `beats` notifies, oldest first, <= 8 each
client.onNotify = (msg) => { if (msg.op === 'beats') plot(msg.data.beats); };

// GPS: satellites are compact tuples in the order of satCols.
if (info.caps.gps) {
  const g = await client.getGps();
  for (const [sys, prn, el, az, snr, used] of g.sats) console.log(sys, prn, el, az, snr, used === 1);
}

// Timebase log: paged. Repeat with offset += entries.length until offset >= total.
let offset = 0, all = [];
for (;;) {
  const page = await client.getTimebaseLog(offset);
  all.push(...page.entries);
  offset += page.entries.length;
  if (page.entries.length === 0 || offset >= page.total) break;
}
```

`setTimebase` accepts `logDeltaPpm` alone (the stored calibration is then untouched), so `ppm` is optional when `logDeltaPpm` is present.

---

## 7. Notes for JS developers

- Prefer **`getState` / `patchState`** over many fine-grained gets/sets; the patch response is the new source of truth.
- Treat sample index fields as **strings**, then use `BigInt(s)` if you need arithmetic.
- Keep subscription rates modest (`metricsHz ≤ 20`) so UAC2 streaming stays healthy.
- Audio recording still goes through the normal Web Audio / `getUserMedia` mic path (UAC2), not WebHID.
- Until H1 firmware ships, `requestDevice` will not find a matching Usage Page `0xFFA0` interface.
