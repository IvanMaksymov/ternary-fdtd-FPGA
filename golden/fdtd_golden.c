/* ================================================================
 * fdtd_golden.c  --  golden model of the ternary FDTD scheme,
 *                    at FPGA word lengths.
 *
 * This is the bit-exact reference the Verilog must reproduce.  It
 * uses the paper's encoder:
 *
 *     v      = x + (3*e1 - e2)/2          N(z) = (1-z^-1)(1-z^-1/2)
 *     s      = clamp(round(v/M), -1, +1)
 *     e1'    = clamp(v - s*M, -2M, +2M)
 *     e2'    = e1
 *
 * with the state accumulator held SEPARATELY from the encoder and
 * never drained:
 *
 *     a += MC * (difference of neighbouring symbols)
 *
 * M and MC are powers of two, so MC*d for d in [-2,+2] is a shift
 * and a select, and the division by two is an arithmetic shift.
 * There is no run-time multiplication anywhere in the update.
 *
 * Build:
 *   gcc -O2 -o gold fdtd_golden.c -lm
 *   gcc -O2 -DM_LOG=13 -DMC_LOG=7 -DNX=32 -DNY=32 -o gold ... -lm
 * ================================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#ifndef NX
#define NX 32
#endif
#ifndef NY
#define NY 32
#endif
#ifndef M_LOG
#define M_LOG 13
#endif
#ifndef MC_LOG
#define MC_LOG 7
#endif
#ifndef NSTEP
#define NSTEP 20000
#endif

/* Source: phase accumulator, PHASE_BITS wide, top LUT_LOG bits index
 * a sine ROM whose entries are round(SRC_AMP * sin(2*pi*k/LUT_N)).
 * SRC_AMP is in STATE units (i.e. field units * M).           */
#ifndef PHASE_BITS
#define PHASE_BITS 16
#endif
#ifndef LUT_LOG
#define LUT_LOG 6
#endif
/* Source normalisation.  Both the drive frequency in cycles per step
 * and the per-step source increment must scale with S = MC/M, so that
 * the PHYSICAL frequency and the injected amplitude per unit PHYSICAL
 * time are invariant as the time step is refined.  Without the second
 * scaling, halving S doubles the injected energy and the convergence
 * study is meaningless.
 *
 *   physical period T_P = (2^PHASE_BITS / PHASE_INC) * S  cells
 *   per-step increment  = SRC_K * MC / M = SRC_K * S      field units  */
#ifndef WAVELEN
#define WAVELEN 16              /* source period in physical time units */
#endif
#ifndef SRC_K
#define SRC_K 3                 /* source amplitude, field units per unit time */
#endif
#define PHASE_INC (((1L << PHASE_BITS) / WAVELEN) >> (M_LOG - MC_LOG))

#define M   (1L << M_LOG)
#define MC  (1L << MC_LOG)
#define LUT_N (1 << LUT_LOG)
#define SIZE (NX * NY)

/* ---------------------------------------------------------------- */
/* diagnostics                                                      */
/* ---------------------------------------------------------------- */

static long max_abs_a  = 0;
static long max_abs_v  = 0;
static long max_abs_e1 = 0;   /* before clamping */
static long n_sat      = 0;   /* ternary saturation  |round(v/M)| > 1 */
static long n_wind     = 0;   /* anti-windup clamp   |e1| > 2M        */
static long n_call     = 0;

#define TRACK(var, val) do { long _t = (val); if (_t < 0) _t = -_t; \
                             if (_t > (var)) (var) = _t; } while (0)

/* ---------------------------------------------------------------- */
/* the encoder                                                      */
/* ---------------------------------------------------------------- */

static inline int tern_encode(long x, long *e1, long *e2)
{
    long v, s;

    /* (3*e1 - e2)/2 : the /2 is an arithmetic right shift, i.e. it
     * truncates toward negative infinity.  The Verilog does the same
     * (>>> 1), so the two agree bit for bit including on negatives. */
    v = x + (((3 * (*e1)) - (*e2)) >> 1);

    TRACK(max_abs_v, v);

    /* round-to-nearest division by M, for signed v */
    if (v >= 0) s =  (v + M / 2) >> M_LOG;
    else        s = -((-v + M / 2) >> M_LOG);

    if (s >  1) { s =  1; n_sat++; }
    if (s < -1) { s = -1; n_sat++; }

    *e2 = *e1;
    *e1 = v - s * M;

    TRACK(max_abs_e1, *e1);

    if (*e1 >  2 * M) { *e1 =  2 * M; n_wind++; }
    if (*e1 < -2 * M) { *e1 = -2 * M; n_wind++; }

    n_call++;
    return (int)s;
}

/* ---------------------------------------------------------------- */
/* fields                                                           */
/* ---------------------------------------------------------------- */

static signed char sEz[SIZE], sHx[SIZE], sHy[SIZE];
static long aEz[SIZE], aHx[SIZE], aHy[SIZE];
static long e1Ez[SIZE], e2Ez[SIZE];
static long e1Hx[SIZE], e2Hx[SIZE];
static long e1Hy[SIZE], e2Hy[SIZE];

/* floating-point reference, identical update order */
static double fEz[SIZE], fHx[SIZE], fHy[SIZE];

static int16_t sine_lut[LUT_N];

#define IDX(i,j) ((j)*NX + (i))

int main(int argc, char **argv)
{
    long n, i, j;
    int  k;
    const double S   = (double)MC / (double)M;
    const long   SRC = (long)SRC_K * MC;
    unsigned     phase = 0;
    const int    SX = NX / 2, SY = NY / 2;
    const int    PX = NX / 4, PY = NY / 4;   /* probe cell */
    FILE        *fp = NULL;

    if (argc > 1) {
        fp = fopen(argv[1], "w");
        if (!fp) { perror("open"); return 1; }
    }

    /* argv[2]: sine LUT as hex, for $readmemh in the Verilog.
     * argv[3]: the derived parameters as a Verilog include file.
     * Emitting both from here makes the golden model the SINGLE source
     * of truth for every number the hardware uses.  PHASE_INC in
     * particular is derived, not chosen, and a hand-copied value that
     * drifts from the reference would silently detune the source. */
    if (argc > 2) {
        FILE *lf = fopen(argv[2], "w");
        int kk;
        if (!lf) { perror("open lut"); return 1; }
        for (kk = 0; kk < LUT_N; kk++) {
            int16_t vv = (int16_t)lround((double)((long)SRC_K * MC)
                                         * sin(2.0 * M_PI * kk / LUT_N));
            fprintf(lf, "%04x\n", (unsigned)(uint16_t)vv);
        }
        fclose(lf);
    }
    if (argc > 3) {
        FILE *pf = fopen(argv[3], "w");
        if (!pf) { perror("open params"); return 1; }
        fprintf(pf, "// generated by fdtd_golden.c -- do not edit\n");
        fprintf(pf, "`define FDTD_NX         %d\n", NX);
        fprintf(pf, "`define FDTD_NY         %d\n", NY);
        fprintf(pf, "`define FDTD_M_LOG      %d\n", M_LOG);
        fprintf(pf, "`define FDTD_MC_LOG     %d\n", MC_LOG);
        fprintf(pf, "`define FDTD_W          %d\n", 18);
        fprintf(pf, "`define FDTD_PHASE_INC  %ld\n", (long)PHASE_INC);
        fprintf(pf, "`define FDTD_NSTEP      %d\n", (int)NSTEP);
        fprintf(pf, "`define FDTD_PX         %d\n", NX/4);
        fprintf(pf, "`define FDTD_PY         %d\n", NY/4);
        fclose(pf);
    }

    for (k = 0; k < LUT_N; k++)
        sine_lut[k] = (int16_t)lround((double)SRC * sin(2.0 * M_PI * k / LUT_N));

    memset(sEz,0,sizeof sEz); memset(sHx,0,sizeof sHx); memset(sHy,0,sizeof sHy);
    memset(aEz,0,sizeof aEz); memset(aHx,0,sizeof aHx); memset(aHy,0,sizeof aHy);
    memset(e1Ez,0,sizeof e1Ez); memset(e2Ez,0,sizeof e2Ez);
    memset(e1Hx,0,sizeof e1Hx); memset(e2Hx,0,sizeof e2Hx);
    memset(e1Hy,0,sizeof e1Hy); memset(e2Hy,0,sizeof e2Hy);
    memset(fEz,0,sizeof fEz); memset(fHx,0,sizeof fHx); memset(fHy,0,sizeof fHy);

    for (n = 0; n < NSTEP; n++) {

        int16_t src = sine_lut[(phase >> (PHASE_BITS - LUT_LOG)) & (LUT_N - 1)];
        phase = (phase + PHASE_INC) & ((1u << PHASE_BITS) - 1u);

        /* ---------------- H sweep ---------------- */
        for (j = 0; j < NY - 1; j++) {
            for (i = 0; i < NX - 1; i++) {
                long id = IDX(i,j);
                int  dx = sEz[id] - sEz[IDX(i,j+1)];   /* Hx += S*(Ez - Ez_up) */
                int  dy = sEz[IDX(i+1,j)] - sEz[id];   /* Hy += S*(Ez_r - Ez)  */

                aHx[id] += (long)MC * dx;
                aHy[id] += (long)MC * dy;
                TRACK(max_abs_a, aHx[id]);
                TRACK(max_abs_a, aHy[id]);

                sHx[id] = (signed char)tern_encode(aHx[id], &e1Hx[id], &e2Hx[id]);
                sHy[id] = (signed char)tern_encode(aHy[id], &e1Hy[id], &e2Hy[id]);

                fHx[id] += S * (fEz[id] - fEz[IDX(i,j+1)]);
                fHy[id] += S * (fEz[IDX(i+1,j)] - fEz[id]);
            }
        }

        /* ---------------- E sweep ---------------- */
        for (j = 1; j < NY - 1; j++) {
            for (i = 1; i < NX - 1; i++) {
                long id = IDX(i,j);
                int  d  = (sHy[id] - sHy[IDX(i-1,j)])
                        - (sHx[id] - sHx[IDX(i,j-1)]);

                aEz[id] += (long)MC * d;
                if (i == SX && j == SY) aEz[id] += src;
                TRACK(max_abs_a, aEz[id]);

                sEz[id] = (signed char)tern_encode(aEz[id], &e1Ez[id], &e2Ez[id]);

                fEz[id] += S * ((fHy[id] - fHy[IDX(i-1,j)])
                              - (fHx[id] - fHx[IDX(i,j-1)]));
                if (i == SX && j == SY) fEz[id] += (double)src / M;
            }
        }

        if (fp)
            fprintf(fp, "%ld %d %ld %.10f\n",
                    n, (int)sEz[IDX(PX,PY)], aEz[IDX(PX,PY)], fEz[IDX(PX,PY)]);
    }

    /* ---------------- report ---------------- */
    printf("# grid %dx%d   M = 2^%d = %ld   MC = 2^%d = %ld   S = %.6f\n",
           NX, NY, M_LOG, (long)M, MC_LOG, (long)MC, S);
    printf("# steps %d   source amp %ld state units (%.4f field units)"
           "   %d steps/period\n",
           (int)NSTEP, SRC, (double)SRC / M, (1 << PHASE_BITS) / PHASE_INC);
    printf("encoder calls        %ld\n", n_call);
    printf("saturation events    %ld  (%.4e %%)\n", n_sat,  100.0*n_sat /n_call);
    printf("anti-windup clamps   %ld  (%.4e %%)\n", n_wind, 100.0*n_wind/n_call);
    printf("max |a|              %ld   -> %d bits signed\n",
           max_abs_a,  (int)ceil(log2((double)max_abs_a  + 1)) + 1);
    printf("max |v|              %ld   -> %d bits signed\n",
           max_abs_v,  (int)ceil(log2((double)max_abs_v  + 1)) + 1);
    printf("max |e1| pre-clamp   %ld   -> %d bits signed\n",
           max_abs_e1, (int)ceil(log2((double)max_abs_e1 + 1)) + 1);
    printf("max |a| / M          %.4f field units\n", (double)max_abs_a / M);

    /* agreement of the state field against the reference, last-quarter RMS */
    {
        double num = 0.0, den = 0.0;
        for (j = 1; j < NY - 1; j++)
            for (i = 1; i < NX - 1; i++) {
                double t = (double)aEz[IDX(i,j)] / M, r = fEz[IDX(i,j)];
                num += (t - r) * (t - r);
                den += r * r;
            }
        printf("final snapshot relative error  %.6f\n", sqrt(num / den));
    }

    if (fp) fclose(fp);
    return 0;
}
