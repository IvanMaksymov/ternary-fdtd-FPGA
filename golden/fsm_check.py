#!/usr/bin/env python3
"""
fsm_check.py -- cycle-accurate replay of fdtd_ternary.v's state machine.

This is NOT a substitute for simulating the RTL.  It exists because the
single highest-risk detail in that module is the BRAM read latency: an
address written in cycle n is applied at the end of n, the memory output
register loads at the end of n+1, and the value is only readable in
n+2.  Get that off by one and the solver still runs, still looks
plausible, and is silently wrong -- it reads a stale neighbour.

So this file transcribes the FSM literally, register by register, with
that two-cycle latency modelled explicitly, and checks the resulting
probe stream against the C golden model.  A mismatch here means the
state sequence in the Verilog is wrong; a match means the sequencing
and the sweep bounds are right, and what remains to be checked on a
real simulator is the Verilog itself rather than the design.

  python3 fsm_check.py build/golden.txt NX M_LOG MC_LOG PHASE_INC LUT
"""
import sys


def run(nx, ny, m_log, mc_log, phase_inc, lut, nstep, px, py):
    M = 1 << m_log
    MC = 1 << mc_log
    SIZE = nx * ny
    SX, SY = nx // 2, ny // 2
    LUT_N = len(lut)
    lut_log = LUT_N.bit_length() - 1
    PHASE_BITS = 16

    # ---- memories -------------------------------------------------
    mem = {k: [0] * SIZE for k in
           ("sEz", "sHx", "sHy", "aEz", "aHx", "aHy",
            "e1Ez", "e2Ez", "e1Hx", "e2Hx", "e1Hy", "e2Hy")}
    names = list(mem)

    # ---- registers ------------------------------------------------
    ra = {k: 0 for k in names}          # read address registers
    q = {k: 0 for k in names}           # memory output registers
    R = dict(state="CLEAR", x=0, y=0, phase=0, clear_idx=0, nstep=0,
             wa=0, we_h=False, we_e=False, we_clear=True,
             probe_valid=False)
    for k in ("ez_cur", "ez_up", "ez_r", "hx_cur", "hx_d", "hy_cur",
              "hy_l", "ahx", "ahy", "aez", "e1hx", "e2hx", "e1hy",
              "e2hy", "e1ez", "e2ez", "ax", "ay", "ae",
              "probe_sym", "probe_state"):
        R[k] = 0
    wd = {}

    def encode(x, e1, e2):
        v = x + ((3 * e1 - e2) >> 1)
        s = (v + M // 2) >> m_log if v >= 0 else -((-v + M // 2) >> m_log)
        s = 1 if s > 1 else (-1 if s < -1 else s)
        e1n = v - s * M
        e1n = 2 * M if e1n > 2 * M else (-2 * M if e1n < -2 * M else e1n)
        return s, e1n

    out = []
    cyc = 0
    LIMIT = 400_000_000

    while len(out) < nstep and cyc < LIMIT:
        cyc += 1
        st = R["state"]
        x, y = R["x"], R["y"]
        cur = y * nx + x
        up = (y + 1) * nx + x
        right = y * nx + (x + 1)
        left = y * nx + (x - 1)
        down = (y - 1) * nx + x

        nra, nq, N, nwd = dict(ra), {}, dict(R), dict(wd)
        N["we_h"] = N["we_e"] = N["we_clear"] = False

        # --- the write port sees the PREVIOUS cycle's wa/we ---------
        if R["we_clear"]:
            for k in names:
                mem[k][R["wa"]] = 0
        else:
            if R["we_h"]:
                for k, vv in (("sHx", wd["sHx"]), ("sHy", wd["sHy"]),
                              ("aHx", wd["aHx"]), ("aHy", wd["aHy"]),
                              ("e1Hx", wd["e1Hx"]), ("e2Hx", wd["e2Hx"]),
                              ("e1Hy", wd["e1Hy"]), ("e2Hy", wd["e2Hy"])):
                    mem[k][R["wa"]] = vv
            if R["we_e"]:
                for k, vv in (("sEz", wd["sEz"]), ("aEz", wd["aEz"]),
                              ("e1Ez", wd["e1Ez"]), ("e2Ez", wd["e2Ez"])):
                    mem[k][R["wa"]] = vv

        # --- memory output registers load from the CURRENT ra -------
        for k in names:
            nq[k] = mem[k][ra[k]]

        src = lut[(R["phase"] >> (PHASE_BITS - lut_log)) & (LUT_N - 1)]

        # --- the state machine --------------------------------------
        if st == "CLEAR":
            N["wa"] = R["clear_idx"]
            N["we_clear"] = True
            if R["clear_idx"] == SIZE - 1:
                N["clear_idx"] = 0
                N["state"] = "IDLE"
            else:
                N["clear_idx"] = R["clear_idx"] + 1

        elif st == "IDLE":
            N["x"] = N["y"] = 0
            N["state"] = "HA"

        elif st == "HA":
            for k in ("sEz", "aHx", "aHy", "e1Hx", "e2Hx", "e1Hy", "e2Hy"):
                nra[k] = cur
            N["state"] = "HB"
        elif st == "HB":
            nra["sEz"] = up
            N["state"] = "HC"
        elif st == "HC":
            N["ez_cur"] = q["sEz"]
            N["ahx"], N["ahy"] = q["aHx"], q["aHy"]
            N["e1hx"], N["e2hx"] = q["e1Hx"], q["e2Hx"]
            N["e1hy"], N["e2hy"] = q["e1Hy"], q["e2Hy"]
            nra["sEz"] = right
            N["state"] = "HD"
        elif st == "HD":
            N["ez_up"] = q["sEz"]; N["state"] = "HE"
        elif st == "HE":
            N["ez_r"] = q["sEz"]; N["state"] = "HF"
        elif st == "HF":
            N["ax"] = R["ahx"] + MC * (R["ez_cur"] - R["ez_up"])
            N["ay"] = R["ahy"] + MC * (R["ez_r"] - R["ez_cur"])
            N["state"] = "HW"
        elif st == "HW":
            shx, e1nx = encode(R["ax"], R["e1hx"], R["e2hx"])
            shy, e1ny = encode(R["ay"], R["e1hy"], R["e2hy"])
            N["wa"] = cur
            nwd.update(sHx=shx, sHy=shy, aHx=R["ax"], aHy=R["ay"],
                       e1Hx=e1nx, e2Hx=R["e1hx"],
                       e1Hy=e1ny, e2Hy=R["e1hy"])
            N["we_h"] = True
            N["state"] = "HN"
        elif st == "HN":
            if x == nx - 2:
                if y == ny - 2:
                    N["x"] = N["y"] = 1; N["state"] = "EA"
                else:
                    N["x"] = 0; N["y"] = y + 1; N["state"] = "HA"
            else:
                N["x"] = x + 1; N["state"] = "HA"

        elif st == "EA":
            for k in ("sHx", "sHy", "aEz", "e1Ez", "e2Ez"):
                nra[k] = cur
            N["state"] = "EB"
        elif st == "EB":
            nra["sHx"] = down; nra["sHy"] = left
            N["state"] = "EC"
        elif st == "EC":
            N["hx_cur"], N["hy_cur"] = q["sHx"], q["sHy"]
            N["aez"] = q["aEz"]
            N["e1ez"], N["e2ez"] = q["e1Ez"], q["e2Ez"]
            N["state"] = "ED"
        elif st == "ED":
            N["hx_d"], N["hy_l"] = q["sHx"], q["sHy"]
            N["state"] = "EE"
        elif st == "EE":
            d = (R["hy_cur"] - R["hy_l"]) - (R["hx_cur"] - R["hx_d"])
            N["ae"] = R["aez"] + MC * d + (src if (x == SX and y == SY) else 0)
            N["state"] = "EW"
        elif st == "EW":
            sez, e1ne = encode(R["ae"], R["e1ez"], R["e2ez"])
            N["wa"] = cur
            nwd.update(sEz=sez, aEz=R["ae"], e1Ez=e1ne, e2Ez=R["e1ez"])
            N["we_e"] = True
            if x == px and y == py:
                N["probe_sym"] = sez
                N["probe_state"] = R["ae"]
            N["state"] = "EN"
        elif st == "EN":
            if x == nx - 2:
                if y == ny - 2:
                    N["state"] = "EMIT"
                else:
                    N["x"] = 1; N["y"] = y + 1; N["state"] = "EA"
            else:
                N["x"] = x + 1; N["state"] = "EA"

        elif st == "EMIT":
            N["phase"] = (R["phase"] + phase_inc) & ((1 << PHASE_BITS) - 1)
            N["nstep"] = R["nstep"] + 1
            N["probe_valid"] = True
            N["state"] = "WAIT"
        elif st == "WAIT":
            # the testbench acknowledges immediately
            out.append((R["probe_sym"], R["probe_state"]))
            N["probe_valid"] = False
            N["x"] = N["y"] = 0
            N["state"] = "HA"

        ra, q, R, wd = nra, nq, N, nwd

    return out, cyc


if __name__ == "__main__":
    gold = sys.argv[1]
    nx = int(sys.argv[2]); m_log = int(sys.argv[3])
    mc_log = int(sys.argv[4]); phase_inc = int(sys.argv[5])
    lut = []
    for ln in open(sys.argv[6]):
        v = int(ln.strip(), 16)
        lut.append(v - 65536 if v >= 32768 else v)

    g = [tuple(int(t) for t in ln.split()[1:3]) for ln in open(gold)]
    px, py = nx // 4, nx // 4

    got, cyc = run(nx, nx, m_log, mc_log, phase_inc, lut, len(g), px, py)

    bad = [i for i, (a, b) in enumerate(zip(got, g)) if a != b]
    print("cycles simulated      %d  (%.0f per step)" % (cyc, cyc / max(len(got), 1)))
    print("steps compared        %d" % len(got))
    print("mismatches            %d" % len(bad))
    for i in bad[:5]:
        print("   step %d: fsm %s  golden %s" % (i, got[i], g[i]))
    print("PASS: FSM sequencing and read latency reproduce the golden model"
          if not bad else "FAILED")
