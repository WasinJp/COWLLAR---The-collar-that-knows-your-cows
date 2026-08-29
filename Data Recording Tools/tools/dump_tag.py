#!/usr/bin/env python3
"""
dump_tag.py — pull recordings off the COWLLAR tag over USB.

    python dump_tag.py --port /dev/ttyACM0 --list
    python dump_tag.py --port /dev/ttyACM0 --all -d ./visit_2026_08_12
    python dump_tag.py --port COM7 --file /rec0001.bin -o cow01.bin

Sends S (stop recording) first so the file is closed and consistent before it is
read. Verifies the checksum the tag reports. Needs: pip install pyserial
"""
import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("needs: pip install pyserial")


def cmd(ser, c, wait=0.3):
    ser.reset_input_buffer()
    ser.write((c + "\n").encode())
    ser.flush()
    time.sleep(wait)
    return ser.read(ser.in_waiting or 1).decode(errors="replace")


def listing(ser):
    ser.reset_input_buffer()
    ser.write(b"L\n"); ser.flush()
    files, t0 = [], time.time()
    buf = ""
    while time.time() - t0 < 5:
        buf += ser.read(ser.in_waiting or 1).decode(errors="replace")
        if "ENDLIST" in buf:
            break
    for line in buf.splitlines():
        if line.startswith("FILE "):
            p = line.split()
            if len(p) >= 3:
                files.append((p[1], int(p[2])))
    return files


def fetch(ser, name, out):
    ser.reset_input_buffer()
    ser.write(f"D {name}\n".encode()); ser.flush()

    header = b""
    t0 = time.time()
    while b"\n" not in header:
        header += ser.read(ser.in_waiting or 1)
        if time.time() - t0 > 5:
            sys.exit(f"no response for {name}")
    line, rest = header.split(b"\n", 1)
    line = line.decode(errors="replace").strip()
    if not line.startswith("BEGIN"):
        sys.exit(f"unexpected reply: {line!r}")
    size = int(line.split()[2])

    data = bytearray(rest)
    t0 = time.time()
    while len(data) < size:
        chunk = ser.read(min(4096, size - len(data)))
        if chunk:
            data += chunk; t0 = time.time()
        elif time.time() - t0 > 10:
            sys.exit(f"\ntimed out at {len(data)}/{size} bytes")
        pct = 100 * len(data) / size
        print(f"\r  {name}  {len(data):>9,}/{size:,} B  {pct:5.1f}%", end="")
    print()

    tail = b""
    t0 = time.time()
    while b"END" not in tail and time.time() - t0 < 5:
        tail += ser.read(ser.in_waiting or 1)
    reported = None
    for tok in tail.decode(errors="replace").split():
        if tok.isdigit():
            reported = int(tok)
    actual = sum(data[:size])
    if reported is not None and reported != actual:
        print(f"  !! checksum mismatch (tag {reported}, got {actual}) — "
              "re-dump this file")
    else:
        print("  checksum ok")

    with open(out, "wb") as f:
        f.write(data[:size])
    print(f"  -> {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--file")
    ap.add_argument("-o", "--out")
    ap.add_argument("-d", "--dir", default=".")
    ap.add_argument("--erase", action="store_true",
                    help="wipe the tag AFTER you have verified the dumps")
    a = ap.parse_args()

    with serial.Serial(a.port, a.baud, timeout=1) as ser:
        time.sleep(2.0)                     # native USB CDC needs a moment
        print(cmd(ser, "S").strip())        # stop recording: closed file, clean read
        print(cmd(ser, "?").strip())

        files = listing(ser)
        if not files:
            print("no recordings on the tag")
            return
        print(f"\n{len(files)} file(s):")
        for n, s in files:
            print(f"  {n:16s} {s:>10,} B  ({s/800/3600:.2f} h)")
        if a.list:
            return

        os.makedirs(a.dir, exist_ok=True)
        targets = files if a.all else [(a.file, 0)] if a.file else []
        if not targets:
            print("\nnothing selected — use --all or --file <name>")
            return
        print()
        for name, _ in targets:
            out = a.out if (a.out and not a.all) else \
                os.path.join(a.dir, os.path.basename(name))
            fetch(ser, name, out)

        if a.erase:
            ans = input("\nerase ALL recordings on the tag? type YES: ")
            if ans.strip() == "YES":
                print(cmd(ser, "X", 2.0).strip())
            else:
                print("not erased")


if __name__ == "__main__":
    main()
