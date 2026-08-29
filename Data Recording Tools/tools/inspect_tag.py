#!/usr/bin/env python3
"""
inspect_tag.py — is the tag actually READING the sensor, or just writing zeros?

    python inspect_tag.py test/rec0001.bin

A non-zero file size only proves the tag wrote something. This checks the
numbers inside it:

  - is the sensor alive at all (does anything vary?)
  - does gravity read ~1.0 g (proves scaling and wiring)
  - is the sample rate really 50 Hz, and is the timing steady
  - did any samples clip, fail, or get dropped
  - an ASCII trace so you can SEE your hand movements in the data
"""
import struct
import sys

HDR = "<8sHHffII8s16s16s8s"
REC = "<hhhhhhBBH"


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: python inspect_tag.py <rec0001.bin>")
    path = sys.argv[1]
    raw = open(path, "rb").read()
    hs, rs = struct.calcsize(HDR), struct.calcsize(REC)

    if len(raw) < hs:
        sys.exit(f"file is only {len(raw)} bytes — no data at all")
    magic, odr, rb, acc, gyr, sess, boot, tag, cow, mount, _ = \
        struct.unpack(HDR, raw[:hs])
    if not magic.startswith(b"COWTAG"):
        sys.exit(f"not a tag file (magic {magic!r}). Tag files are COWTAG; the "
                 "Cardputer logger writes COWLOG and needs decode_log.py")

    n = (len(raw) - hs) // rs
    print(f"file    : {magic.decode()}  session {sess}")
    tg = tag.split(b"\x00")[0].decode(errors="replace")
    cw = cow.split(b"\x00")[0].decode(errors="replace")
    print(f"tag/cow : {tg} / {cw}")
    print(f"samples : {n:,}  ({n/odr/60:.2f} min at {odr} Hz nominal)")
    if n < 10:
        sys.exit("\n!! almost no samples — the sampler is not running")

    r = [struct.unpack_from(REC, raw, hs + i*rs) for i in range(n)]
    ax = [v[0]/acc for v in r]; ay = [v[1]/acc for v in r]; az = [v[2]/acc for v in r]
    dts = [v[7] for v in r[1:]]
    flags = [v[6] for v in r]

    def stats(v):
        m = sum(v)/len(v)
        sd = (sum((x-m)**2 for x in v)/len(v)) ** .5
        return m, sd, min(v), max(v)

    print("\n--- is the sensor alive? ---")
    dead = True
    for name, v in (("ax", ax), ("ay", ay), ("az", az)):
        m, sd, lo, hi = stats(v)
        print(f"  {name}  mean {m:+7.3f} g   sd {sd:6.3f}   range {lo:+6.2f}..{hi:+6.2f}")
        if sd > 0.005:
            dead = False
    if dead:
        print("  !! NOTHING VARIES. Sensor not being read, or it is not moving at all.")
    else:
        print("  ok — values vary, the sensor is being read")

    mag = [(x*x+y*y+z*z) ** .5 for x, y, z in zip(ax, ay, az)]
    mm, msd, mlo, mhi = stats(mag)
    print(f"\n--- gravity check ---")
    print(f"  |a| mean {mm:.3f} g  (expect ~1.00 at rest)")
    if 0.85 <= mm <= 1.20:
        print("  ok — scaling and wiring are right")
    elif mm < 0.1:
        print("  !! near zero: sensor returning nothing")
    else:
        print("  !! unexpected — check the full-scale setting in tag_config.h")

    print(f"\n--- timing ---")
    if dts:
        s = sorted(dts)
        p50, p99 = s[len(s)//2], s[int(len(s)*0.99)]
        late = sum(1 for d in dts if d > 2*(1000//odr))
        real = 1000.0/p50 if p50 else 0
        print(f"  median {p50} ms (= {real:.1f} Hz), p99 {p99} ms")
        print(f"  late: {late} of {len(dts)} ({100*late/len(dts):.2f}%)")
        if p50 == 1000//odr and late < 0.01*len(dts):
            print("  ok — clean, steady sampling")
        else:
            print("  !! jitter — frequency features will be degraded")

    imufail = sum(1 for f in flags if f & 0x01)
    overrun = sum(1 for f in flags if f & 0x02)
    clipped = sum(1 for v in r if max(abs(v[0]), abs(v[1]), abs(v[2])) > 32000)
    print(f"\n--- faults ---")
    print(f"  IMU read failures : {imufail}")
    print(f"  buffer overruns   : {overrun}")
    print(f"  clipped samples   : {clipped}")
    if not (imufail or overrun or clipped):
        print("  ok — none")

    print(f"\n--- |a| over time (each row ~{n//40/odr:.1f} s) ---")
    print("  0g" + " "*22 + "1g" + " "*22 + "2g")
    step = max(1, n//40)
    for i in range(0, n - step, step):
        w = mag[i:i+step]
        avg = sum(w)/len(w)
        col = min(49, int(avg*25))
        bar = " "*col + "#"
        print(f"  {i/odr:6.1f}s |{bar}")
    print("\n  Flat line = it never moved. Wiggles = your hand. If you waved it")
    print("  and this is flat, the file is not recording what you think it is.")


if __name__ == "__main__":
    main()
