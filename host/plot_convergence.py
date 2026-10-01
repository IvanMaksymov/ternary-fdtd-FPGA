#!/usr/bin/env python3
"""
plot_convergence.py -- convergence figure from the hardware sweep.

Reads every fpga_state_mc*.txt produced by `make sweep`, together with
the fpga_params_mc*.vh and fpga_golden_mc*.txt saved beside it, and
produces the three-panel figure:

  A  relative error at the probe against the Courant number, log-log,
     with an O(S) guide and the fitted order
  B  the probe waveform against the floating-point reference
  C  the probe spectrum with the hard-wall cavity eigenfrequencies

The Courant number for each capture comes from its OWN parameter file,
never from a constant in this script.  Five byte-identical captures once
went unnoticed because the analysis assumed the S it expected rather
than reading the S that was built.

    python3 host/plot_convergence.py
"""
import glob
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SW, HW, REF, GUIDE = "#0072B2", "#D55E00", "#009E73", "#8a8a8a"
INK, MUTED = "#1a1a1a", "#5c5c5c"

plt.rcParams.update({
    "font.size": 9, "axes.labelsize": 9.5, "axes.titlesize": 9.5,
    "axes.edgecolor": "#c8c8c8", "axes.linewidth": 0.8,
    "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.labelcolor": INK,
    "figure.facecolor": "white", "axes.facecolor": "white",
    "legend.frameon": False, "legend.fontsize": 8.5,
})

def params(fn):
    d = {}
    for ln in open(fn):
        m = re.match(r"`define\s+FDTD_(\w+)\s+(-?\d+)", ln.strip())
        if m:
            d[m.group(1)] = int(m.group(2))
    d["S"] = 2.0 ** (d["MC_LOG"] - d["M_LOG"])
    d["M"] = 2 ** d["M_LOG"]
    d["PER"] = 65536 // d["PHASE_INC"]
    return d


runs = []
for fn in sorted(glob.glob("fpga_state_mc*.txt")):
    mc = fn.split("mc")[1].split(".")[0]
    pf, gf = "fpga_params_mc%s.vh" % mc, "fpga_golden_mc%s.txt" % mc
    if not (os.path.exists(pf) and os.path.exists(gf)):
        print("skipping %s: need %s and %s beside it" % (fn, pf, gf))
        continue
    P = params(pf)
    hw_, gc_ = np.loadtxt(fn), np.loadtxt(gf)
    n = min(len(hw_), len(gc_))
    hw_, gc_ = hw_[:n], gc_[:n]
    nz = np.nonzero(gc_[:, 2])[0]
    if len(nz) == 0:
        print("skipping %s: reference never reaches the probe" % fn)
        continue
    skip = nz[0] + 4 * P["PER"]
    avail = n - skip
    if avail <= 0:
        print("skipping %s: too short after the transient" % fn)
        continue
    exact = np.array_equal(hw_[:, 1], gc_[:, 1]) and \
            np.array_equal(hw_[:, 2], gc_[:, 2])
    runs.append(dict(S=P["S"], hw=hw_, gc=gc_, P=P,
                     skip=skip, avail=avail, exact=exact))

if len(runs) < 2:
    sys.exit("need at least two sweep points; run `make sweep` first")

# FIXED PHYSICAL TIME.  The capture is a fixed NUMBER OF STEPS, so the
# physical time T = N*S differs by the full range of S across the sweep
# -- a factor of 16 here.  Comparing those directly measures error
# accumulated over different durations and steepens the fitted order
# (1.18 instead of 0.90).  The convergence study holds N*S constant, so
# every run is truncated to the shortest common physical time.
T_common = min(d["avail"] * d["S"] for d in runs)
print("  common physical time: %.1f  (%.1f source periods)\n"
      % (T_common, T_common / (runs[0]["P"]["PER"] * runs[0]["S"])))

for d in runs:
    P = d["P"]
    nkeep = min(d["avail"], int(round(T_common / P["S"])))
    m = (nkeep // P["PER"]) * P["PER"]
    d["m"] = m
    a = d["hw"][d["skip"]:d["skip"] + m, 2] / P["M"]
    r = d["gc"][d["skip"]:d["skip"] + m, 3]
    d["err"] = np.sqrt(np.mean((a - r) ** 2)) / np.sqrt(np.mean(r ** 2))
    print("  S = 1/%-5d  T = %7.1f  %5.1f periods  rel err %.4f   "
          "bit-exact vs C: %s"
          % (1 / P["S"], m * P["S"], m / P["PER"], d["err"], d["exact"]))

runs = [d for d in runs if d["m"] > 0]
runs.sort(key=lambda d: d["S"])
S = np.array([d["S"] for d in runs])
rms = np.array([d["err"] for d in runs])
if len(set(np.round(S, 12))) < len(S):
    sys.exit("ERROR: two captures report the same S -- the sweep did not "
             "rebuild.  Check the bitstream checksums printed by `make sweep`.")

fine = runs[0]
M, S_HW, PER = fine["P"]["M"], fine["S"], fine["P"]["PER"]
hw, gc = fine["hw"], fine["gc"]

fig, ax = plt.subplots(1, 3, figsize=(11.5, 3.5))

# ---- A: convergence -------------------------------------------------
a = ax[0]
g = np.array([S.min() * 0.7, S.max() * 1.4])
a.loglog(g, rms[0] * (g / S[0]), "--", color=GUIDE, lw=1.2, zorder=1)
a.annotate(r"$O(S)$", (g[0] * 1.5, rms[0] * (g[0] * 1.5 / S[0]) * 1.5),
           color=GUIDE, fontsize=8.5)
a.loglog(S, rms, "-o", color=SW, lw=2, ms=6, mec="white", mew=1.2,
         label="measured on FPGA", zorder=3)
ex = [i for i, d in enumerate(runs) if d["exact"]]
if ex:
    a.loglog(S[ex], rms[ex], "o", color=HW, ms=11, mec="white", mew=1.8,
             label="bit-exact against the C model", zorder=4)
p = np.polyfit(np.log(S), np.log(rms), 1)
r2 = np.corrcoef(np.log(S), np.log(rms))[0, 1] ** 2
a.text(0.97, 0.06, "$p = %.2f$\n$R^2 = %.3f$" % (p[0], r2),
       transform=a.transAxes, va="bottom", ha="right", fontsize=9, color=INK)
a.set_xlabel("Courant number $S$")
a.set_ylabel("relative error at the probe")
a.set_xlim(S.min() * 0.6, S.max() * 1.7)
a.set_title("Convergence", loc="left", pad=8)
a.grid(True, which="both", color="#ebebeb", lw=0.7, zorder=0)
a.set_axisbelow(True)
a.legend(loc="upper left")

# ---- B: waveform ----------------------------------------------------
b = ax[1]
# Clamp the waveform window to what both records actually contain:
# the reference run can be shorter than the capture, and indexing past
# its end produces an empty array and an unhelpful matplotlib error.
nz = np.nonzero(gc[:, 2])[0][0]
navail = min(len(gc), len(hw))
span = min(2 * PER, max(0, navail - nz - 1))
k0 = min(nz + 8 * PER, navail - span)
k0 = max(k0, nz)
w = slice(k0, k0 + span)
t = (np.arange(k0, k0 + span) - k0) * S_HW
b.plot(t, gc[w, 3], "-", color=REF, lw=2.2, label="floating-point Yee", zorder=2)
b.plot(t, hw[w, 2] / M, "-", color=HW, lw=1.2, label="FPGA state field", zorder=3)
b.set_xlabel("physical time")
b.set_ylabel("$E_z$  (units of $A$)")
b.set_title("Probe waveform,  $S = 1/%d$" % round(1 / S_HW), loc="left", pad=8)
b.grid(True, color="#ebebeb", lw=0.7, zorder=0)
b.set_axisbelow(True)
b.legend(loc="upper right")

# ---- C: spectrum ----------------------------------------------------
cx = ax[2]
skip, n = fine["skip"], fine["m"]
x = hw[skip:skip + n, 2] / M
x = x - x.mean()
win = 0.5 * (1 - np.cos(2 * np.pi * np.arange(n) / (n - 1)))
P = np.abs(np.fft.rfft(x * win)) / (n * win.mean() / 2)
f = np.fft.rfftfreq(n, S_HW)
sel = f < 0.13
NXG = fine["P"]["NX"]
modes = sorted({np.hypot(m, nn) / (2 * (NXG - 1))
                for m in range(1, 6) for nn in range(1, 6)})
for j, fm in enumerate([x for x in modes if x < 0.115]):
    cx.axvline(fm, color="#dcdcdc", lw=0.9, zorder=0,
               label="hard-wall modes $f_{mn}$" if j == 0 else None)
cx.semilogy(f[sel], np.maximum(P[sel], 1e-6), "-", color=HW, lw=1.3,
            label="FPGA state field", zorder=3)
cx.axvline(1 / 16, color=INK, ls="--", lw=1.2, zorder=2)
cx.annotate("$f_0$", (1 / 16, P[sel].max() * 1.5), color=INK,
            fontsize=9, ha="center")

cx.set_xlabel("frequency (physical units)")
cx.set_ylabel("amplitude")
cx.set_title("Probe spectrum,  $S = 1/%d$" % round(1 / S_HW), loc="left", pad=8)
cx.set_xlim(0, 0.13)
cx.grid(True, which="major", axis="y", color="#ebebeb", lw=0.7, zorder=0)
cx.set_axisbelow(True)
cx.legend(loc="lower left")

fig.tight_layout(pad=1.4)
fig.savefig("fpga_convergence.png", dpi=190, bbox_inches="tight")
print("wrote fpga_convergence.png")
