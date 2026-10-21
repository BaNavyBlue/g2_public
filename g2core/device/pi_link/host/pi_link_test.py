#!/usr/bin/env python3
"""Host-side test tool for the g2core Pi link, protocol v0 (see ../PROTOCOL.md).

Examples (on the Pi, J17 wired to GPIO14/15):
    pi_link_test.py ping                     # one PING, prints the round trip
    pi_link_test.py ping -n 1000 -s 64       # 1000 PINGs with 64-byte payloads
    pi_link_test.py status                   # one STATUS, decoded
    pi_link_test.py status --watch 0.2       # STATUS every 0.2 s until Ctrl-C
    pi_link_test.py errors                   # sends bad frames, checks the board counts them
"""

import argparse
import os
import struct
import sys
import time
import zlib

import serial

VERSION = 0
PING, STATUS_REQ, PONG, STATUS, NAK = 0x01, 0x02, 0x81, 0x82, 0xFF
STATUS_FMT = "<BBBBHBxf4fIHHHH"
STATES = ["initializing", "ready", "alarm", "program stop", "program end", "run", "hold",
          "probe", "cycle", "homing", "jog", "interlock", "shutdown", "panic"]


def cobs_encode(data):
    out = bytearray([0])
    code_pos, code = 0, 1
    for b in data:
        if b == 0:
            out[code_pos] = code
            code_pos, code = len(out), 1
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_pos] = code
                code_pos, code = len(out), 1
                out.append(0)
    out[code_pos] = code
    return bytes(out)


def cobs_decode(data):
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            raise ValueError("bad COBS")
        out += data[i:i + code - 1]
        i += code - 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def build_frame(ftype, seq, payload=b"", version=VERSION, crc_xor=0):
    body = struct.pack("<BBH", version, ftype, seq) + payload
    return cobs_encode(body + struct.pack("<I", zlib.crc32(body) ^ crc_xor)) + b"\x00"


class Link:
    def __init__(self, port, baud, timeout):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.seq = 0

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def send_raw(self, raw):
        self.ser.write(raw)

    def recv(self):
        """Returns (type, seq, payload), or None on timeout. Raises ValueError on a bad frame."""
        raw = self.ser.read_until(b"\x00")
        if not raw.endswith(b"\x00"):
            return None
        frame = cobs_decode(raw[:-1])
        if len(frame) < 8:
            raise ValueError("short frame (%d bytes)" % len(frame))
        if zlib.crc32(frame[:-4]) != struct.unpack("<I", frame[-4:])[0]:
            raise ValueError("bad CRC")
        version, ftype, seq = struct.unpack("<BBH", frame[:4])
        if version != VERSION:
            raise ValueError("version %d" % version)
        return ftype, seq, frame[4:-4]

    def request(self, ftype, payload=b""):
        seq = self.next_seq()
        self.ser.reset_input_buffer()
        t0 = time.perf_counter()
        self.send_raw(build_frame(ftype, seq, payload))
        reply = self.recv()
        dt = time.perf_counter() - t0
        if reply is None:
            raise TimeoutError("no reply to type 0x%02X seq %d" % (ftype, seq))
        if reply[1] != seq:
            raise ValueError("seq %d, expected %d" % (reply[1], seq))
        return reply, dt

    def status(self):
        (ftype, _, payload), _ = self.request(STATUS_REQ)
        if ftype != STATUS or len(payload) != 40:
            raise ValueError("expected STATUS, got type 0x%02X, %d bytes" % (ftype, len(payload)))
        f = struct.unpack(STATUS_FMT, payload)
        return {
            "stat": f[0], "safe": f[1], "estp": f[2], "homed": f[3], "inputs": f[4],
            "spc": f[5], "sps": f[6], "mpo": f[7:11], "uptime_ms": f[11],
            "ok": f[12], "crc_err": f[13], "framing_err": f[14], "overflow": f[15],
        }


def print_status(s):
    state = STATES[s["stat"]] if s["stat"] < len(STATES) else "?"
    homed = "".join(a for i, a in enumerate("XYZA") if s["homed"] & (1 << i)) or "-"
    inputs = " ".join("in%d" % (i + 1) for i in range(16) if s["inputs"] & (1 << i)) or "-"
    print("stat %d (%s)  safe %d  estp %d  homed %s  spc %d  sps %.0f"
          % (s["stat"], state, s["safe"], s["estp"], homed, s["spc"], s["sps"]))
    print("mpo X %.3f  Y %.3f  Z %.3f  A %.3f" % s["mpo"])
    print("inputs active: %s" % inputs)
    print("uptime %.1f s  frames ok %d  crc_err %d  framing_err %d  overflow %d"
          % (s["uptime_ms"] / 1000, s["ok"], s["crc_err"], s["framing_err"], s["overflow"]))


def cmd_ping(link, args):
    rtts, failures = [], 0
    for i in range(args.count):
        payload = os.urandom(args.size)
        try:
            (ftype, _, echo), dt = link.request(PING, payload)
            if ftype != PONG or echo != payload:
                raise ValueError("bad PONG (type 0x%02X, %d bytes)" % (ftype, len(echo)))
            rtts.append(dt)
        except (TimeoutError, ValueError) as e:
            failures += 1
            print("ping %d: %s" % (i, e))
    if rtts:
        rtts.sort()
        print("%d/%d ok, %d-byte payload, round trip ms: min %.2f  median %.2f  max %.2f"
              % (len(rtts), args.count, args.size, rtts[0] * 1e3, rtts[len(rtts) // 2] * 1e3,
                 rtts[-1] * 1e3))
    return 0 if failures == 0 else 1


def cmd_status(link, args):
    if args.watch is None:
        print_status(link.status())
        return 0
    try:
        while True:
            print_status(link.status())
            print()
            time.sleep(args.watch)
    except KeyboardInterrupt:
        return 0


def cmd_errors(link, args):
    """Each bad frame must be dropped (no reply) and counted; the link must keep working."""
    before = link.status()
    cases = [
        ("bad CRC", build_frame(PING, 1, b"x", crc_xor=1), "crc_err"),
        ("bad COBS", b"\x05\x01\x00", "framing_err"),
        ("too short", cobs_encode(b"\x00\x01\x00") + b"\x00", "framing_err"),
        ("too long", cobs_encode(bytes(range(1, 200))) + b"\x00", "framing_err"),
        ("wrong version", build_frame(PING, 1, b"x", version=7), "framing_err"),
    ]
    expected = {"crc_err": 0, "framing_err": 0}
    ok = True
    for name, raw, counter in cases:
        link.ser.reset_input_buffer()
        link.send_raw(raw)
        expected[counter] += 1
        reply = link.recv()
        if reply is not None:
            print("FAIL %s: got a reply (type 0x%02X)" % (name, reply[0]))
            ok = False
    # NAK cases: these are valid frames, so they get an answer
    for name, ftype, payload, reason in [("unknown type", 0x42, b"", 1),
                                         ("PING too long", PING, bytes(65), 2),
                                         ("STATUS_REQ with payload", STATUS_REQ, b"x", 2)]:
        (rtype, _, p), _ = link.request(ftype, payload)
        if rtype != NAK or p != bytes([reason, ftype]):
            print("FAIL %s: expected NAK %d, got type 0x%02X %s" % (name, reason, rtype, p.hex()))
            ok = False
    after = link.status()
    for counter, n in expected.items():
        got = (after[counter] - before[counter]) & 0xFFFF
        if got != n:
            print("FAIL %s went up by %d, expected %d" % (counter, got, n))
            ok = False
    if after["overflow"] != before["overflow"]:
        print("FAIL overflow counter changed")
        ok = False
    print("errors: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyAMA0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-t", "--timeout", type=float, default=0.5, help="reply timeout, s")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("ping")
    p.add_argument("-n", "--count", type=int, default=1)
    p.add_argument("-s", "--size", type=int, default=8, help="payload bytes, 0-64")
    p = sub.add_parser("status")
    p.add_argument("--watch", type=float, metavar="SECONDS")
    sub.add_parser("errors")
    args = ap.parse_args()

    link = Link(args.port, args.baud, args.timeout)
    return {"ping": cmd_ping, "status": cmd_status, "errors": cmd_errors}[args.cmd](link, args)


if __name__ == "__main__":
    sys.exit(main())
