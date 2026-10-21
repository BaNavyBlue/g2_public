#!/usr/bin/env python3
"""Host-side test tool for the g2core Pi link (see ../PROTOCOL.md and ../PROTOCOL-v1-draft.md).

Examples (on the Pi, J17 wired to GPIO14/15; add -b 2000000 for a 2 Mbaud board):
    pi_link_test.py ping                     # one PING, prints the round trip
    pi_link_test.py ping -n 1000 -s 64       # 1000 PINGs with 64-byte payloads
    pi_link_test.py status                   # one STATUS, decoded
    pi_link_test.py status --watch 0.2       # STATUS every 0.2 s until Ctrl-C
    pi_link_test.py errors                   # sends bad frames, checks the board counts them
    pi_link_test.py diag                     # timing and segment counters
    pi_link_test.py enable                   # take control (one ENABLE); the watchdog then hands it back
    pi_link_test.py disable                  # give control back
    pi_link_test.py stream -s 10             # ENABLE, 1 kHz zero-velocity MOTION for 10 s, DISABLE
    pi_link_test.py stream -s 3 --gap 1:50   # ...and stop sending for 50 ms at 1 s (watchdog test)
    pi_link_test.py safety ack|clear         # E-stop acknowledge / clear alarm, while USB has control
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
DIAG_REQ, ENABLE, MOTION, DISABLE, SAFETY, DIAG, FEEDBACK = 0x03, 0x10, 0x11, 0x12, 0x13, 0x83, 0x91
STATUS_FMT = "<BBBBHBxf4fIHHHH"
FEEDBACK_FMT = "<BBBBI4iHHBBHBBxxf"
DIAG_FMT = "<HHIIHHHHHxx"
STATES = ["initializing", "ready", "alarm", "program stop", "program end", "run", "hold",
          "probe", "cycle", "homing", "jog", "interlock", "shutdown", "panic"]
OWNERS = ["USB", "PI", "STOPPING"]
RESULTS = ["ok", "not idle", "E-stop", "alarm", "bad parameter", "not in control", "already in control"]
REASONS = ["none", "DISABLE", "watchdog", "E-stop", "door timeout", "alarm", "bad frame", "USB stop", "fault"]

# Desktop CNC defaults, in steps: X/Y 200/mm, Z 400/mm, A 17.78/deg
ENABLE_DEFAULTS = dict(watchdog_ms=5, segment_us=500, hold_ms=250,
                       max_vel=(17000.0, 17000.0, 13400.0, 600.0),          # ~84, 84, 33 mm/s, 33 deg/s
                       stop_decel=(200000.0, 200000.0, 400000.0, 17780.0))  # 1000 mm/s^2, 1000 deg/s^2


def name(table, i):
    return table[i] if i < len(table) else str(i)


def enable_payload(watchdog_ms, segment_us, hold_ms, max_vel, stop_decel):
    return struct.pack("<HHHH4f4f12x", watchdog_ms, segment_us, hold_ms, 0, *max_vel, *stop_decel)


def motion_payload(vel=(0.0, 0.0, 0.0, 0.0), spindle=0, rpm=0.0):
    return struct.pack("<4fBBHf", *vel, spindle, 0, 0, rpm)


def decode_feedback(payload):
    f = struct.unpack(FEEDBACK_FMT, payload)
    return {"result": f[0], "owner": f[1], "reason": f[2], "flags": f[3], "time_us": f[4],
            "pos": f[5:9], "queued_seq": f[9], "stale": f[10], "safe": f[11], "estp": f[12],
            "inputs": f[13], "spc": f[14], "stat": f[15], "sps": f[16]}


def format_feedback(fb):
    flags = [n for b, n in ((1, "hold"), (2, "clamped"), (4, "stale"), (8, "motion-locked")) if fb["flags"] & b]
    return ("result %s  owner %s  last release %s  stat %d  safe %d  estp %d  pos %s  flags %s"
            % (name(RESULTS, fb["result"]), name(OWNERS, fb["owner"]), name(REASONS, fb["reason"]),
               fb["stat"], fb["safe"], fb["estp"], list(fb["pos"]), ",".join(flags) or "-"))


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
        self.timeout_s = timeout
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

    def request(self, ftype, payload=b"", skip_others=False):
        """skip_others: drop replies to earlier frames (e.g. MOTION still in flight) until ours."""
        seq = self.next_seq()
        self.ser.reset_input_buffer()
        t0 = time.perf_counter()
        self.send_raw(build_frame(ftype, seq, payload))
        while True:
            reply = self.recv()
            dt = time.perf_counter() - t0
            if reply is None:
                raise TimeoutError("no reply to type 0x%02X seq %d" % (ftype, seq))
            if reply[1] == seq:
                return reply, dt
            if not skip_others or dt > self.timeout_s:
                raise ValueError("seq %d, expected %d" % (reply[1], seq))

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


def request_feedback(link, ftype, payload=b""):
    (rtype, _, p), _ = link.request(ftype, payload, skip_others=True)
    if rtype != FEEDBACK or len(p) != 40:
        raise ValueError("expected FEEDBACK, got type 0x%02X, %d bytes" % (rtype, len(p)))
    return decode_feedback(p)


def print_diag(link):
    (rtype, _, p), _ = link.request(DIAG_REQ)
    if rtype != DIAG or len(p) != 24:
        raise ValueError("expected DIAG, got type 0x%02X" % rtype)
    d = struct.unpack(DIAG_FMT, p)
    print("exec max %d us (last %d)  main-loop gap max %d us  segments %d  stale %d  watchdog trips %d  "
          "segment %d ticks (%.1f us)  replies dropped %d  underruns %d" % (d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[6] * 1e6 / 61000, d[7], d[8]))
    return d


def cmd_diag(link, args):
    print_diag(link)
    return 0


def cmd_enable(link, args):
    fb = request_feedback(link, ENABLE, enable_payload(**ENABLE_DEFAULTS))
    print(format_feedback(fb))
    return 0 if fb["result"] == 0 else 1


def cmd_disable(link, args):
    fb = request_feedback(link, DISABLE)
    print(format_feedback(fb))
    return 0 if fb["result"] == 0 else 1


def cmd_safety(link, args):
    fb = request_feedback(link, SAFETY, bytes([{"ack": 1, "clear": 2}[args.action]]))
    print(format_feedback(fb))
    return 0 if fb["result"] == 0 else 1


def cmd_stream(link, args):
    """ENABLE, then MOTION (zero velocity) every period without waiting for replies, then DISABLE.
    Prints every change of owner / release reason / result, and a summary."""
    fb = request_feedback(link, ENABLE, enable_payload(**ENABLE_DEFAULTS))
    print("ENABLE: " + format_feedback(fb))
    if fb["result"] != 0:
        return 1
    gap_at, gap_ms = (float(x) for x in args.gap.split(":")) if args.gap else (None, 0)

    ser = link.ser
    ser.timeout = 0
    period = args.period_ms / 1000.0
    t0 = next_t = time.perf_counter()
    sent, replies, results, buf = 0, 0, {}, bytearray()
    last_key, send_times, lat, late = None, {}, [], 0
    while True:
        now = time.perf_counter()
        el = now - t0
        if el >= args.seconds:
            break
        in_gap = gap_at is not None and gap_at <= el < gap_at + gap_ms / 1000.0
        if not in_gap:
            seq = link.next_seq()
            send_times[seq] = now
            ser.write(build_frame(MOTION, seq, motion_payload()))
            sent += 1
        buf += ser.read(4096)
        while b"\x00" in buf:
            raw, _, rest = bytes(buf).partition(b"\x00")
            buf = bytearray(rest)
            try:
                frame = cobs_decode(raw)
            except ValueError:
                continue
            if len(frame) < 8 or zlib.crc32(frame[:-4]) != struct.unpack("<I", frame[-4:])[0]:
                continue
            _, rtype, rseq = struct.unpack("<BBH", frame[:4])
            if rtype != FEEDBACK:
                continue
            fb = decode_feedback(frame[4:-4])
            replies += 1
            results[fb["result"]] = results.get(fb["result"], 0) + 1
            if rseq in send_times:
                lat.append(time.perf_counter() - send_times.pop(rseq))
            key = (fb["owner"], fb["reason"], fb["result"])
            if key != last_key:
                print("%8.3f s  " % (time.perf_counter() - t0) + format_feedback(fb))
                last_key = key
        next_t += period
        if time.perf_counter() > next_t + period:
            late += 1                       # this host fell behind by more than a period
            next_t = time.perf_counter()
        while time.perf_counter() < next_t:
            pass
    ser.timeout = link.timeout_s
    fb = request_feedback(link, DISABLE)        # at once: the watchdog would end control in 5 ms
    print("DISABLE: " + format_feedback(fb))
    lat.sort()
    print("sent %d MOTION, %d FEEDBACK  results %s  host late %d" %
          (sent, replies, {name(RESULTS, k): v for k, v in results.items()}, late))
    if lat:
        print("reply latency ms: median %.2f  99%% %.2f  max %.2f" %
              (lat[len(lat) // 2] * 1e3, lat[int(len(lat) * 0.99)] * 1e3, lat[-1] * 1e3))
    print_diag(link)
    return 0


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
    sub.add_parser("diag")
    sub.add_parser("enable")
    sub.add_parser("disable")
    p = sub.add_parser("safety")
    p.add_argument("action", choices=["ack", "clear"])
    p = sub.add_parser("stream")
    p.add_argument("-s", "--seconds", type=float, default=10)
    p.add_argument("--period-ms", type=float, default=1.0)
    p.add_argument("--gap", metavar="AT_S:MS", help="stop sending for MS milliseconds at AT_S seconds")
    args = ap.parse_args()

    link = Link(args.port, args.baud, args.timeout)
    return {"ping": cmd_ping, "status": cmd_status, "errors": cmd_errors, "diag": cmd_diag,
            "enable": cmd_enable, "disable": cmd_disable, "safety": cmd_safety,
            "stream": cmd_stream}[args.cmd](link, args)


if __name__ == "__main__":
    sys.exit(main())
