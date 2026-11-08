# WebHID example (Timegrapher control)

Minimal browser-side sketch for the HID control plane. Normative details: [`../HID_CONTROL_PLANE.md`](../HID_CONTROL_PLANE.md). Types: [`timegrapher_hid.d.ts`](timegrapher_hid.d.ts).

**Requires:** Chromium-based browser, HTTPS or `localhost`, user gesture for `requestDevice`. Firmware with `CFG_TUD_HID=1` (current builds). Interactive page: [`example.html`](example.html).

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
const MAX_JSON = 2048;

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
      const buf = new Uint8Array(5 + MAX_CHUNK);
      const more = i + 1 < fragCount ? FLAG_MORE : 0;
      buf[0] = more;
      buf[1] = seq;
      buf[2] = i;
      buf[3] = fragCount;
      buf[4] = chunk.length;
      buf.set(chunk, 5);
      reports.push(buf.subarray(0, 5 + chunk.length));
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
  getEvents(limit = 8) { return this.request('getEvents', { limit }); }
  subscribe(opts) { return this.request('subscribe', opts); }
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

## 5. Notes for JS developers

- Prefer **`getState` / `patchState`** over many fine-grained gets/sets; the patch response is the new source of truth.
- Treat sample index fields as **strings**, then use `BigInt(s)` if you need arithmetic.
- Keep subscription rates modest (`metricsHz ≤ 20`) so UAC2 streaming stays healthy.
- Audio recording still goes through the normal Web Audio / `getUserMedia` mic path (UAC2), not WebHID.
- Until H1 firmware ships, `requestDevice` will not find a matching Usage Page `0xFFA0` interface.
