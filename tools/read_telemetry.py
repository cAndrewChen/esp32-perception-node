"""Reads the telemetry stream, resynchronises on the magic bytes, and prints decoded packets.

Usage:
    python tools/read_telemetry.py                      # rfc2217://localhost:4000 (Wokwi)
    python tools/read_telemetry.py --port COM5          # a real serial port
    python tools/read_telemetry.py --file capture.bin   # a saved byte stream
    python tools/read_telemetry.py --selftest           # no hardware, no pyserial needed

The packet layout is documented in firmware/main/packet.h.
"""
import argparse
import struct
import sys
import time

MAGIC = b"\xAA\x55"
PACKET_LEN = 23
# little-endian: magic(2s) flags(B) t_ms(I) dist(H) pitch roll yaw servo obj_x obj_y (6 x h) crc(H)
FMT = "<2sBIHhhhhhhH"
assert struct.calcsize(FMT) == PACKET_LEN


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Decoder:
    """Feed it bytes in any chunking. It yields only packets whose CRC checks out."""

    def __init__(self):
        self.buf = bytearray()
        self.good = 0
        self.bad_crc = 0
        self.skipped = 0

    def feed(self, data: bytes):
        self.buf += data
        while True:
            start = self.buf.find(MAGIC)
            if start < 0:
                # keep a trailing 0xAA: it may be the first half of a magic split across reads
                keep = 1 if self.buf.endswith(MAGIC[:1]) else 0
                self.skipped += len(self.buf) - keep
                del self.buf[: len(self.buf) - keep]
                return
            if start > 0:
                self.skipped += start
                del self.buf[:start]
            if len(self.buf) < PACKET_LEN:
                return
            raw = bytes(self.buf[:PACKET_LEN])
            if crc16_ccitt(raw[2:21]) == struct.unpack_from("<H", raw, 21)[0]:
                del self.buf[:PACKET_LEN]
                self.good += 1
                yield self.parse(raw)
            else:
                # Not a real packet (the magic appeared by chance, or the data is damaged).
                # Drop just this magic and look for the next one.
                self.bad_crc += 1
                del self.buf[:1]

    @staticmethod
    def parse(raw: bytes) -> dict:
        _, flags, t_ms, dist, pitch, roll, yaw, servo, ox, oy, _ = struct.unpack(FMT, raw)
        return {
            "t_s": t_ms / 1000.0,
            "obj_found": bool(flags & 1),
            "dist_valid": bool(flags & 2),
            "dist_fresh": bool(flags & 4),
            "dist_cm": dist / 10.0,
            "pitch": pitch / 100.0, "roll": roll / 100.0, "yaw": yaw / 100.0,
            "servo": servo / 100.0,
            "obj_x": ox / 100.0, "obj_y": oy / 100.0,
        }


def make_packet(flags=7, t_ms=1234, dist=1000, pitch=-1234, roll=250, yaw=17999, servo=-4500, ox=-1550, oy=300) -> bytes:
    body = struct.pack("<BIHhhhhhh", flags, t_ms, dist, pitch, roll, yaw, servo, ox, oy)
    return MAGIC + body + struct.pack("<H", crc16_ccitt(body))


def selftest() -> int:
    assert crc16_ccitt(b"123456789") == 0x29B1, "CRC check value"
    pkt = make_packet()
    assert len(pkt) == PACKET_LEN
    # joins mid-stream, with garbage, a corrupted packet, a fake magic and a split read
    stream = b"boot log text\r\n\xAA" + pkt[5:] + pkt + b"\xAA\x55" + b"\x00" * 7 + pkt[:10] + b"\xFF" + pkt[11:] + pkt
    d = Decoder()
    got = []
    for i in range(0, len(stream), 7):          # awkward 7-byte chunks
        got += list(d.feed(stream[i:i + 7]))
    assert len(got) == 2, f"expected 2 good packets, got {len(got)} (good={d.good}, bad={d.bad_crc})"
    assert got[0]["dist_cm"] == 100.0 and got[0]["servo"] == -45.0 and got[0]["obj_x"] == -15.5
    print(f"selftest ok: CRC check value matches, {d.good} good packets, {d.bad_crc} rejected, {d.skipped} bytes skipped")
    return 0


def open_source(args):
    if args.file:
        f = open(args.file, "rb")
        return lambda: f.read(64) or None
    import serial  # pip install pyserial
    url = args.port or "rfc2217://localhost:4000"
    ser = serial.serial_for_url(url, baudrate=115200, timeout=0.1)
    return lambda: ser.read(256)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--file")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--rate", type=float, default=5.0, help="printed lines per second")
    args = ap.parse_args()
    if args.selftest:
        return selftest()

    read = open_source(args)
    d = Decoder()
    last_print = 0.0
    last_stats = time.time()
    print("waiting for packets...")
    while True:
        data = read()
        if data is None:
            break
        for p in d.feed(data):
            now = time.time()
            if now - last_print >= 1.0 / args.rate:
                last_print = now
                dist = f"{p['dist_cm']:6.1f} cm{'' if p['dist_fresh'] else ' (held)'}" if p["dist_valid"] else "   no echo yet"
                obj = f"({p['obj_x']:6.1f},{p['obj_y']:6.1f}) px" if p["obj_found"] else "      not found"
                print(f"t={p['t_s']:8.2f}s  dist={dist}  pitch={p['pitch']:7.2f} roll={p['roll']:7.2f} "
                      f"yaw={p['yaw']:7.2f}  servo={p['servo']:7.2f}  obj={obj}")
        if time.time() - last_stats > 5:
            last_stats = time.time()
            print(f"[stats] good={d.good} bad_crc={d.bad_crc} skipped_bytes={d.skipped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
