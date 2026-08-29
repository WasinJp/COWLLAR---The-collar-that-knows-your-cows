#!/usr/bin/env python3
"""
merge_labels.py — join the on-cow tag recording with the Cardputer label log.

    python merge_labels.py rec0001.bin LBL0001.CSV -o cow01.csv

The two devices have independent clocks. This tool finds the tap-sync spikes in
the accelerometer trace, matches them to the M presses in the label log, and
emits a combined CSV in the format tune_thresholds.py already reads.

IT REFUSES TO MERGE when the start and end sync points disagree. That is the
whole point of tapping twice. A tool that always returns something would hand
you a plausible-looking dataset built on a wrong offset, and you would never
find out. Use --force only if you understand exactly why they disagree.
"""
import argparse
import csv
import struct
import sys

HDR_FMT = "<8sHHffII8s16s16s8s"
REC_FMT = "<hhhhhhBBH"
LABELS = ["UNKNOWN", "LYING", "STANDING", "RUMINATE",
          "GRAZE", "WALK", "DRINK", "OTHER"]


# ------------------------------------------------------------------ reading ---
def read_tag(path):
    raw = open(path, "rb").read()
    hs, rs = struct.calcsize(HDR_FMT), struct.calcsize(REC_FMT)
    magic, odr, rb, acc, gyr, sess, boot, tag, cow, mount, _ = \
        struct.unpack(HDR_FMT, raw[:hs])
    if not magic.startswith(b"COWTAG"):
        sys.exit(f"{path}: not a tag recording (magic {magic!r})")
    if rb != rs:
        sys.exit(f"{path}: record size {rb} != {rs}; firmware/tool mismatch")
    body = raw[hs:]
    n = len(body) // rs
    recs = [struct.unpack_from(REC_FMT, body, i * rs) for i in range(n)]
    meta = dict(odr=odr, acc=acc, gyr=gyr, session=sess,
                tag=tag.split(b"\0")[0].decode(errors="replace"),
                cow=cow.split(b"\0")[0].decode(errors="replace"),
                mount=mount.split(b"\0")[0].decode(errors="replace"))
    return meta, recs


def read_labels(path):
    ev = []
    with open(path) as f:
        for row in csv.DictReader(f):
            ev.append((int(row["t_ms"]), row["event"], int(row["code"]), row["name"]))
    # UNDO marks the preceding LABEL as bad rather than deleting it: the file
    # stays append-only and auditable, and we drop the pair here.
    out, i = [], 0
    while i < len(ev):
        if i + 1 < len(ev) and ev[i + 1][1] == "UNDO" and ev[i][1] == "LABEL":
            i += 2
            continue
        if ev[i][1] != "UNDO":
            out.append(ev[i])
        i += 1
    return out


# --------------------------------------------------------------- tap finding ---
def find_taps(recs, odr, acc_lsb, window_s=60.0):
    """Locate tap clusters near each end of the recording.

    A deliberate tap is far sharper than anything a cow does: we look for the
    jerk (first difference of |a|) crossing a high robust threshold, then group
    hits within 2 s into clusters and keep clusters of >=2 hits.
    """
    n = len(recs)
    mag = [((r[0]**2 + r[1]**2 + r[2]**2) ** 0.5) / acc_lsb for r in recs]
    jerk = [abs(mag[i] - mag[i - 1]) for i in range(1, n)]
    if not jerk:
        return [], []

    s = sorted(jerk)
    med = s[len(s) // 2]
    mad = sorted(abs(j - med) for j in jerk)[len(s) // 2] or 1e-6
    thr = med + 12 * mad          # deliberately strict: taps are unmistakable

    def clusters(lo, hi):
        hits = [i for i in range(lo, min(hi, len(jerk))) if jerk[i] > thr]
        out, cur = [], []
        for h in hits:
            if cur and h - cur[-1] > 2 * odr:
                out.append(cur); cur = []
            cur.append(h)
        if cur:
            out.append(cur)
        return [c for c in out if len(c) >= 2]

    w = int(window_s * odr)
    return clusters(0, w), clusters(max(0, n - w), n)


# --------------------------------------------------------------------- main ---
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tagfile")
    ap.add_argument("labelfile")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--tolerance", type=float, default=1.0,
                    help="max start/end sync disagreement in seconds")
    ap.add_argument("--offset", type=float,
                    help="skip detection, force this offset (tag_t = label_t + offset)")
    ap.add_argument("--force", action="store_true",
                    help="merge anyway on sync disagreement (you had better know why)")
    a = ap.parse_args()

    meta, recs = read_tag(a.tagfile)
    events = read_labels(a.labelfile)
    odr = meta["odr"]
    n = len(recs)
    print(f"tag   : {meta['tag']} cow={meta['cow']} mount={meta['mount']} "
          f"{n} samples ({n/odr/60:.1f} min @ {odr} Hz)")

    marks = [e for e in events if e[1] == "MARK"]
    labels = [e for e in events if e[1] == "LABEL"]
    print(f"labels: {len(labels)} label events, {len(marks)} sync marks")

    overrun = sum(1 for r in recs if r[7] & 0x02)
    imufail = sum(1 for r in recs if r[7] & 0x01)
    if overrun:
        print(f"  !! {overrun} samples flagged OVERRUN — the tag lost data")
    if imufail:
        print(f"  !! {imufail} IMU read failures ({100*imufail/max(n,1):.1f}%)")

    # ---- establish the offset
    if a.offset is not None:
        offset = a.offset
        print(f"offset: {offset:+.3f} s (forced)")
    else:
        if len(marks) < 2:
            sys.exit("\nERROR: need 2 sync marks (start and end). Found "
                     f"{len(marks)}.\nUse --offset if you can work it out another "
                     "way, but the honest answer is that this session cannot be\n"
                     "aligned reliably. Do the tap ritual next time.")
        head, tail = find_taps(recs, odr, meta["acc"])
        if not head or not tail:
            sys.exit("\nERROR: could not find tap clusters in the tag data.\n"
                     "Either the taps were too gentle or the tag was not held "
                     "against the Cardputer.\nInspect the file and use --offset "
                     "if you can identify them by eye.")
        # Both ends must reference the SAME moment in the ritual — the first
        # tap of the burst. Taking the first tap at one end and the last at the
        # other builds the burst duration into the disagreement and makes a
        # perfectly good sync look broken.
        tag_start = head[0][0] / odr
        tag_end = tail[-1][0] / odr
        lbl_start = marks[0][0] / 1000.0
        lbl_end = marks[-1][0] / 1000.0

        off_start = tag_start - lbl_start
        off_end = tag_end - lbl_end
        disagree = abs(off_start - off_end)
        print(f"sync  : start offset {off_start:+.3f} s, "
              f"end offset {off_end:+.3f} s, disagreement {disagree:.3f} s")

        if disagree > a.tolerance:
            msg = (f"\nERROR: sync points disagree by {disagree:.3f} s "
                   f"(tolerance {a.tolerance:.1f} s).\n"
                   "Something is wrong: a missed tap, a mark pressed at the wrong "
                   "moment, or a clock\nthat stalled. Merging now would produce a "
                   "dataset that looks fine and is\nsilently mislabelled — which "
                   "is worse than no dataset at all.\n"
                   "Inspect both files before using --force.")
            if not a.force:
                sys.exit(msg)
            print(msg + "\n--force given, continuing anyway.")
        offset = (off_start + off_end) / 2
        print(f"offset: {offset:+.3f} s (mean of both ends)")

    # ---- build the per-sample label track
    track = [(e[0] / 1000.0 + offset, e[2]) for e in labels]
    track.sort()
    if not track:
        sys.exit("no LABEL events — nothing to merge")

    acc, gyr = meta["acc"], meta["gyr"]
    idx, cur = 0, 0
    counts = {}
    with open(a.out, "w") as f:
        f.write("t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                "label,label_name,marker,dt_ms\n")
        for i, r in enumerate(recs):
            t = i / odr
            while idx < len(track) and track[idx][0] <= t:
                cur = track[idx][1]; idx += 1
            counts[cur] = counts.get(cur, 0) + 1
            nm = LABELS[cur] if cur < len(LABELS) else f"L{cur}"
            f.write(f"{t:.4f},{r[0]/acc:.5f},{r[1]/acc:.5f},{r[2]/acc:.5f},"
                    f"{r[3]/gyr:.3f},{r[4]/gyr:.3f},{r[5]/gyr:.3f},"
                    f"{cur},{nm},0,{r[7] and 0 or r[7]}\n")

    print(f"\nwrote {a.out}")
    print("coverage:")
    for lb in sorted(counts):
        nm = LABELS[lb] if lb < len(LABELS) else f"L{lb}"
        mins = counts[lb] / odr / 60
        flag = "" if lb == 0 or mins >= 20 else "   <- thin, want >=20 min"
        print(f"  {nm:9s} {mins:6.1f} min  {'#' * int(40*counts[lb]/n)}{flag}")
    print("\nnext: python tune_thresholds.py " + a.out +
          f" --src-odr {odr} --odr 25")


if __name__ == "__main__":
    main()
