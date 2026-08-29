#!/usr/bin/env python3
"""
tune_thresholds.py — fit the collar's decision-tree constants to labelled cow data.

    python tune_thresholds.py cow01.csv cow02.csv --odr 25 --window 256 --hop 64

What it does:
  1. Reads decoded logger CSVs (from decode_log.py), decimates to the collar's ODR.
  2. Recomputes the EXACT 8 features the firmware computes, in the same order.
  3. Groups windows by the human label and prints a per-class table.
  4. Fits each of the four tree thresholds in tree order, the same way the tree
     actually evaluates them, and prints the resulting confusion matrix.
  5. Prints a ready-to-paste block of #defines.

It also reports the NORMALISED band-energy variant alongside the raw one, because
demo_config.h already flags unnormalised band energy as a known weakness: with
raw energies a vigorous walk can out-score a real rumination inside the
rumination band. Whichever variant separates the classes better on real cattle
is the one to ship — and now you have the evidence either way.

Needs: numpy. Optional: matplotlib (--plot), scikit-learn (--tree).
"""
import argparse
import csv
import sys
from collections import Counter

import numpy as np

# Collar label space. The logger records finer labels; these merge into it.
CLASSES = ["REST", "RUMINATE", "GRAZE", "WALK", "ACTIVE"]
MERGE = {
    "LYING": "REST", "STANDING": "REST",
    "RUMINATE": "RUMINATE",
    "GRAZE": "GRAZE", "DRINK": "GRAZE",     # head-down, similar posture
    "WALK": "WALK",
    "OTHER": "ACTIVE",
    "UNKNOWN": None,                        # dropped, never guessed
}

BAND_RUMIN = (0.8, 2.0)
BAND_GAIT = (1.8, 3.5)


# ----------------------------------------------------------------- loading ---
def load_csv(path):
    t, xyz, lab = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_s"]))
            xyz.append((float(row["ax_g"]), float(row["ay_g"]), float(row["az_g"])))
            lab.append(row["label_name"])
    return np.array(t), np.array(xyz, dtype=np.float64), np.array(lab)


def decimate(xyz, lab, src_odr, dst_odr):
    if src_odr == dst_odr:
        return xyz, lab
    if src_odr % dst_odr:
        sys.exit(f"source {src_odr} Hz is not an integer multiple of {dst_odr} Hz")
    k = src_odr // dst_odr
    # Anti-alias before dropping samples. Skipping this is a real bug: energy
    # above the new Nyquist folds back into the rumination band and inflates it.
    win = np.ones(k) / k
    sm = np.stack([np.convolve(xyz[:, i], win, mode="same") for i in range(3)], axis=1)
    return sm[::k], lab[::k]


# ---------------------------------------------------------------- features ---
def features(win, odr):
    """Mirrors extract_features() in main.cpp, feature-for-feature."""
    m = win.mean(axis=0)
    d = win - m
    f = np.zeros(8)
    f[0:3] = np.sqrt((d ** 2).mean(axis=0))              # per-axis std
    f[3] = np.abs(d).sum(axis=1).mean()                  # SMA on DC-removed axes

    mag = np.linalg.norm(win, axis=1)
    mag_dc = mag - mag.mean()
    f[4] = np.sum(mag_dc[1:] * mag_dc[:-1] < 0) / len(win)   # ZCR

    spec = np.abs(np.fft.rfft(mag_dc * np.hamming(len(win))))
    bin_hz = odr / len(win)

    def band(lo, hi):
        klo, khi = max(int(lo / bin_hz), 1), int(hi / bin_hz)
        return float((spec[klo:khi + 1] ** 2).sum())

    f[5] = band(*BAND_RUMIN)
    f[6] = band(*BAND_GAIT)
    k = 1 + int(np.argmax(spec[1:]))
    f[7] = k * bin_hz
    total = float((spec[1:] ** 2).sum()) + 1e-12
    return f, total


def windows(xyz, lab, odr, wlen, hop, purity=0.9):
    """Yield (features, class, total_energy) for label-pure windows only.

    A window straddling a graze->walk transition belongs to neither class, and
    including it drags both fitted thresholds toward each other. Dropping mixed
    windows costs a few percent of the data and buys clean boundaries.
    """
    out = []
    dropped_mixed = dropped_unknown = 0
    for s in range(0, len(xyz) - wlen + 1, hop):
        seg_lab = lab[s:s + wlen]
        c = Counter(MERGE.get(x) for x in seg_lab)
        top, n = c.most_common(1)[0]
        if top is None:
            dropped_unknown += 1
            continue
        if n / wlen < purity:
            dropped_mixed += 1
            continue
        f, tot = features(xyz[s:s + wlen], odr)
        out.append((f, top, tot))
    return out, dropped_mixed, dropped_unknown


# --------------------------------------------------------------- threshold ---
def best_split(pos, neg, higher_is_pos):
    """Optimal 1-D threshold by balanced accuracy. Returns (thr, balanced_acc)."""
    if len(pos) == 0 or len(neg) == 0:
        return None, 0.0
    cands = np.unique(np.concatenate([pos, neg]))
    if len(cands) > 1:
        cands = (cands[1:] + cands[:-1]) / 2
    best, best_score = cands[0], -1
    for t in cands:
        if higher_is_pos:
            tp, tn = (pos > t).mean(), (neg <= t).mean()
        else:
            tp, tn = (pos < t).mean(), (neg >= t).mean()
        s = (tp + tn) / 2
        if s > best_score:
            best, best_score = t, s
    return float(best), float(best_score)


def fit_tree(F, y, norm):
    """Fit the four constants in the order the tree evaluates them.

    Order matters: TH_ACTIVE_SMA is fitted only on the samples that survive the
    REST gate, because that is the only population it ever sees at runtime.
    Fitting each gate on the whole dataset would give constants that look good
    on paper and misbehave on the device.
    """
    e_r = F[:, 5] / F[:, 8] if norm else F[:, 5]
    e_g = F[:, 6] / F[:, 8] if norm else F[:, 6]
    res, scores = {}, {}

    m = np.ones(len(y), bool)
    res["TH_REST_SMA"], scores["TH_REST_SMA"] = best_split(
        F[m & (y == "REST"), 3], F[m & (y != "REST"), 3], higher_is_pos=False)

    m = y != "REST"
    res["TH_ACTIVE_SMA"], scores["TH_ACTIVE_SMA"] = best_split(
        F[m & (y == "ACTIVE"), 3], F[m & (y != "ACTIVE"), 3], higher_is_pos=True)

    m = ~np.isin(y, ["REST", "ACTIVE"])
    res["TH_WALK_GAIT_E"], scores["TH_WALK_GAIT_E"] = best_split(
        e_g[m & (y == "WALK")], e_g[m & (y != "WALK")], higher_is_pos=True)

    m = np.isin(y, ["RUMINATE", "GRAZE"])
    res["TH_RUMIN_BAND_E"], scores["TH_RUMIN_BAND_E"] = best_split(
        e_r[m & (y == "RUMINATE")], e_r[m & (y != "RUMINATE")], higher_is_pos=True)

    return res, scores, e_r, e_g


def classify(F, e_r, e_g, th):
    out = np.empty(len(F), dtype=object)
    for i in range(len(F)):
        if th["TH_REST_SMA"] is not None and F[i, 3] < th["TH_REST_SMA"]:
            out[i] = "REST"
        elif th["TH_ACTIVE_SMA"] is not None and F[i, 3] > th["TH_ACTIVE_SMA"]:
            out[i] = "ACTIVE"
        elif th["TH_WALK_GAIT_E"] is not None and e_g[i] > th["TH_WALK_GAIT_E"]:
            out[i] = "WALK"
        elif th["TH_RUMIN_BAND_E"] is not None and e_r[i] > th["TH_RUMIN_BAND_E"]:
            out[i] = "RUMINATE"
        else:
            out[i] = "GRAZE"
    return out


def confusion(y, pred):
    corner = "true/pred"
    print(f"{corner:>10s}" + "".join(f"{c[:6]:>8s}" for c in CLASSES) + "   recall")
    for t in CLASSES:
        m = y == t
        if not m.sum():
            print(f"{t:>10s}" + "".join(f"{'-':>8s}" for _ in CLASSES) + "      n/a")
            continue
        row = [(pred[m] == p).sum() for p in CLASSES]
        print(f"{t:>10s}" + "".join(f"{v:>8d}" for v in row) +
              f"   {row[CLASSES.index(t)] / m.sum() * 100:5.1f}%")
    print(f"\noverall accuracy: {(y == pred).mean() * 100:.1f}%  (n={len(y)} windows)")


# -------------------------------------------------------------------- main ---
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+")
    ap.add_argument("--src-odr", type=int, default=50, help="rate in the CSV")
    ap.add_argument("--odr", type=int, default=25, help="collar rate to fit for")
    ap.add_argument("--window", type=int, default=256)
    ap.add_argument("--hop", type=int, default=64)
    ap.add_argument("--purity", type=float, default=0.9)
    ap.add_argument("--holdout", help="filename substring to hold out for testing")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--tree", action="store_true",
                    help="also fit a real sklearn decision tree for comparison")
    a = ap.parse_args()

    allw, src = [], []
    for p in a.csvs:
        _, xyz, lab = load_csv(p)
        xyz, lab = decimate(xyz, lab, a.src_odr, a.odr)
        w, mixed, unk = windows(xyz, lab, a.odr, a.window, a.hop, a.purity)
        print(f"{p}: {len(w)} clean windows "
              f"({mixed} mixed-label, {unk} unknown dropped)")
        allw += w
        src += [p] * len(w)

    if not allw:
        sys.exit("no usable windows — check labels and window length")

    F = np.array([[*w[0], w[2]] for w in allw])     # col 8 = total spectral energy
    y = np.array([w[1] for w in allw])
    src = np.array(src)

    print(f"\nwindow {a.window} @ {a.odr} Hz = {a.window/a.odr:.2f} s, hop {a.hop}")
    print("\n--- per-class features (median, [p10 p90]) ---")
    hdr = ["SMA", "ZCR", "f0_Hz", "E_rumin", "E_gait", "E_rumin_n", "E_gait_n"]
    cols = [3, 4, 7, 5, 6, None, None]
    print(f"{'class':>9s}{'n':>7s}" + "".join(f"{h:>22s}" for h in hdr))
    for c in CLASSES:
        m = y == c
        if not m.sum():
            print(f"{c:>9s}{0:>7d}   (no labelled windows — you did not record this)")
            continue
        cells = []
        for h, ci in zip(hdr, cols):
            v = (F[m, ci] if ci is not None
                 else F[m, 5 if h == "E_rumin_n" else 6] / F[m, 8])
            cells.append(f"{np.median(v):8.3f} [{np.percentile(v,10):5.2f} "
                         f"{np.percentile(v,90):5.2f}]")
        print(f"{c:>9s}{m.sum():>7d}" + "".join(f"{s:>22s}" for s in cells))

    for norm in (False, True):
        tag = "NORMALISED band energy" if norm else "RAW band energy (as shipped)"
        print(f"\n=== {tag} ===")
        if a.holdout:
            tr = np.array([a.holdout not in s for s in src])
            th, sc, _, _ = fit_tree(F[tr], y[tr], norm)
            e_r = F[:, 5] / F[:, 8] if norm else F[:, 5]
            e_g = F[:, 6] / F[:, 8] if norm else F[:, 6]
            print(f"fitted on {tr.sum()} windows, tested on {(~tr).sum()} held out "
                  f"({a.holdout})")
            confusion(y[~tr], classify(F[~tr], e_r[~tr], e_g[~tr], th))
        else:
            th, sc, e_r, e_g = fit_tree(F, y, norm)
            confusion(y, classify(F, e_r, e_g, th))
            print("(no --holdout: these numbers are optimistic, the same data "
                  "fitted and scored)")

        print("\n  gate quality (balanced accuracy of each split):")
        for k, v in sc.items():
            flag = "" if v > 0.85 else ("   <- weak, this gate is guessing" if v < 0.7 else "   <- marginal")
            print(f"    {k:<18s} {v*100:5.1f}%{flag}")

        print("\n  paste into demo_config.h / collar_config.h:")
        for k, v in th.items():
            print(f"    #define {k:<18s} {v:.4f}f" if v is not None
                  else f"    #define {k:<18s} /* NOT FITTED - missing class */")

    if a.tree:
        try:
            from sklearn.tree import DecisionTreeClassifier, export_text
            clf = DecisionTreeClassifier(max_depth=4, min_samples_leaf=20)
            clf.fit(F[:, :8], y)
            names = ["std_x", "std_y", "std_z", "SMA", "ZCR", "E_rumin", "E_gait", "f0"]
            print("\n=== unconstrained tree (for comparison only) ===")
            print(export_text(clf, feature_names=names))
            print("If this tree is much more accurate than the 4-gate one, the")
            print("hand-designed structure is costing you real performance and is")
            print("worth revisiting. If it is similar, the simple tree is fine and")
            print("far easier to defend and to run on a coin cell.")
        except ImportError:
            print("\n(--tree needs scikit-learn; pip install scikit-learn)")

    if a.plot:
        try:
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(1, 3, figsize=(15, 4))
            for c in CLASSES:
                m = y == c
                if not m.sum():
                    continue
                axes[0].scatter(F[m, 3], F[m, 6], s=6, alpha=.5, label=c)
                axes[1].scatter(F[m, 5], F[m, 6], s=6, alpha=.5, label=c)
                axes[2].scatter(F[m, 7], F[m, 3], s=6, alpha=.5, label=c)
            for ax, (xl, yl) in zip(axes, [("SMA", "E_gait"),
                                           ("E_rumin", "E_gait"),
                                           ("f0 (Hz)", "SMA")]):
                ax.set_xlabel(xl); ax.set_ylabel(yl); ax.set_yscale("symlog")
            axes[0].legend()
            plt.tight_layout(); plt.savefig("features.png", dpi=120)
            print("\nwrote features.png — look at it before trusting any number above")
        except ImportError:
            print("\n(--plot needs matplotlib)")


if __name__ == "__main__":
    main()
