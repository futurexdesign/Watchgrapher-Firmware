"""Minimal Timegrapher control-plane client for bench tests (USB HID and BLE).

Framing per docs/HID_CONTROL_PLANE.md: frames of a 5-byte header
(flags, msgSeq, fragIndex, fragCount, payloadLen) + up to 58 JSON bytes from
the host. HID: report id 1, frames zero-padded to 63 bytes. BLE: unpadded
frames; ``BleClient.connect(frame_max=255)`` opts in to large device-to-host
frames (BLE_CONTROL_PLANE.md section 4.3a).

Used by scripts/bench/validate_clips_hw.py; see
docs/handoff/TIMING_VALIDATION_HANDOFF.md. Needs ``hidapi`` (and ``bleak`` for
BLE): ``python3 -m venv .venv && .venv/bin/pip install hidapi bleak``.

Every request needs control of the device (``claim`` with ``takeover=True``
first if another app holds it); ``patchState`` takes its changes under a
``patch`` key: ``request("patchState", patch={...})``. Always check the
reply's ``ok``.
"""
import asyncio
import json
import threading
import time

import hid

VID, PID, USAGE_PAGE = 0xCAFE, 0x4003, 0xFFA0
SERVICE = "c137e7c0-92b4-4d9e-87e0-9ba757169016"
RX_UUID = "c137e7c1-92b4-4d9e-87e0-9ba757169016"
TX_UUID = "c137e7c2-92b4-4d9e-87e0-9ba757169016"
INFO_UUID = "c137e7c3-92b4-4d9e-87e0-9ba757169016"
CHUNK = 58
FLAG_MORE = 0x01
FLAG_NOTIFY = 0x02


def fragments(payload: bytes, seq: int, pad: bool):
    """Splits a JSON payload into frames."""
    chunks = [payload[i:i + CHUNK] for i in range(0, len(payload), CHUNK)] or [b""]
    out = []
    for i, c in enumerate(chunks):
        flags = FLAG_MORE if i + 1 < len(chunks) else 0
        f = bytes([flags, seq & 0xFF, i, len(chunks), len(c)]) + c
        if pad:
            f = f + bytes(63 - len(f))
        out.append(f)
    return out


class Reassembler:
    """Collects frames into complete messages (dict)."""

    def __init__(self):
        self.seq = None
        self.parts = {}
        self.count = 0

    def push(self, frame: bytes):
        if len(frame) < 5:
            return None
        flags, seq, idx, cnt, ln = frame[:5]
        data = frame[5:5 + ln]
        if idx == 0 or seq != self.seq:
            self.seq, self.parts, self.count = seq, {}, cnt
        self.parts[idx] = data
        if len(self.parts) == self.count:
            raw = b"".join(self.parts[i] for i in range(self.count))
            self.seq = None
            return json.loads(raw.decode())
        return None


class _Base:
    def __init__(self):
        self._seq = 0
        self._id = 0
        self.notifies = []
        self._replies = {}
        self._cv = threading.Condition()

    def _on_message(self, msg):
        with self._cv:
            if msg.get("notify"):
                self.notifies.append((time.time(), msg))
            elif "id" in msg:
                self._replies[msg["id"]] = msg
            self._cv.notify_all()

    def _next(self):
        self._seq = (self._seq + 1) & 0xFF
        self._id += 1
        return self._seq, self._id


class HidClient(_Base):
    """USB HID link (blocking API, reader thread)."""

    def __init__(self):
        super().__init__()
        path = next(d["path"] for d in hid.enumerate(VID, PID) if d["usage_page"] == USAGE_PAGE)
        self.dev = hid.device()
        self.dev.open_path(path)
        self.dev.set_nonblocking(False)
        self._stop = False
        self._rx = Reassembler()
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        while not self._stop:
            try:
                d = self.dev.read(64, 200)
            except OSError:
                return
            if not d:
                continue
            b = bytes(d)
            if b[0] == 1 and len(b) == 64:
                b = b[1:]
            m = self._rx.push(b)
            if m is not None:
                self._on_message(m)

    def request(self, op, timeout=3.0, **body):
        seq, rid = self._next()
        msg = {"v": 1, "id": rid, "op": op, **body}
        for f in fragments(json.dumps(msg, separators=(",", ":")).encode(), seq, True):
            self.dev.write(bytes([1]) + f)
        deadline = time.time() + timeout
        with self._cv:
            while rid not in self._replies:
                left = deadline - time.time()
                if left <= 0:
                    raise TimeoutError(f"{op}: no reply")
                self._cv.wait(left)
            return self._replies.pop(rid)

    def close(self):
        self._stop = True
        self._t.join(1)
        self.dev.close()


class BleClient(_Base):
    """BLE link via bleak (async API)."""

    def __init__(self, client):
        super().__init__()
        self.client = client
        self._rx = Reassembler()
        self.frames_in = 0
        self._aev = {}

    @classmethod
    async def connect(cls, name_prefix="TG-", timeout=10.0, frame_max=None):
        from bleak import BleakClient, BleakScanner
        dev = await BleakScanner.find_device_by_filter(
            lambda d, ad: (d.name or "").startswith(name_prefix) or SERVICE in (ad.service_uuids or []),
            timeout=timeout)
        if dev is None:
            raise RuntimeError("no TG device found")
        c = BleakClient(dev)
        await c.connect()
        self = cls(c)
        self.device = dev
        self.loop = asyncio.get_running_loop()
        await c.start_notify(TX_UUID, self._on_notify)
        if frame_max:
            r = await self.request("hello", frameMax=frame_max)
            self.frame_max = r.get("result", {}).get("link", {}).get("frameMax", 63)
        return self

    def _on_notify(self, _h, data: bytearray):
        self.frames_in += 1
        m = self._rx.push(bytes(data))
        if m is not None:
            self._on_message(m)
            ev = self._aev.get(m.get("id"))
            if ev is not None:
                ev.set()

    async def info(self):
        return json.loads((await self.client.read_gatt_char(INFO_UUID)).decode())

    async def request(self, op, timeout=5.0, **body):
        seq, rid = self._next()
        ev = asyncio.Event()
        self._aev[rid] = ev
        msg = {"v": 1, "id": rid, "op": op, **body}
        for f in fragments(json.dumps(msg, separators=(",", ":")).encode(), seq, False):
            await self.client.write_gatt_char(RX_UUID, f, response=True)
        try:
            await asyncio.wait_for(ev.wait(), timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(f"{op}: no reply") from None
        finally:
            self._aev.pop(rid, None)
        with self._cv:
            return self._replies.pop(rid)

    async def close(self):
        try:
            await self.client.disconnect()
        except Exception:
            pass
