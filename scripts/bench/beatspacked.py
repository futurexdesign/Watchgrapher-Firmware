"""Reference decoder for the packed beats block (plan §14.2, format 1)."""
import base64
import struct

HDR = struct.Struct("<BBIIfQi")      # fmt n seq0 gridId nominalUs sample0 k0 = 26 B
BEAT = struct.Struct("<BBIBhiHH")    # dSeq flags dSample frac dK u amp lift = 17 B
STATUS = {0: "accepted", 1: "started", 2: "rejected"}


def decode(p: str, source_hz: float = 48000.0):
    raw = base64.b64decode(p)
    fmt, n, seq, grid, nominal, sample, k = HDR.unpack_from(raw, 0)
    if fmt != 1:
        raise ValueError(f"unknown packed beats format {fmt}")
    if len(raw) != HDR.size + n * BEAT.size:
        raise ValueError("length mismatch")
    out = []
    for i in range(n):
        d_seq, flags, d_sample, frac, d_k, u, amp, lift = BEAT.unpack_from(raw, HDR.size + i * BEAT.size)
        seq += d_seq
        sample += d_sample
        k += d_k
        out.append({"seq": seq, "gridId": grid, "sampleIndex": sample,
                    "tS": (sample + frac / 256) / source_hz, "k": k, "parity": flags & 1,
                    "uUs": u / 10, "nominalUs": nominal, "amplitudeDeg": amp / 100,
                    "amplitudeValid": bool(flags & 2), "liftMs": lift / 1000,
                    "status": STATUS.get((flags >> 2) & 3, "rejected")})
    return out
