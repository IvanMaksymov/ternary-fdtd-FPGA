/* enc_vectors.c -- exhaustive-ish test vectors for tern_encode.
 *
 * Writes lines:  x e1 e2 s e1_next sat wind
 * as signed decimals, covering the ordinary range, both signs, the
 * saturation boundary and the anti-windup boundary.  The Verilog unit
 * test replays these and must match every field.
 *
 *   gcc -O2 -DM_LOG=13 -o encvec enc_vectors.c -lm
 */
#include <stdio.h>
#include <stdlib.h>

#ifndef M_LOG
#define M_LOG 13
#endif
#define M (1L << M_LOG)

int main(int argc, char **argv)
{
    FILE *f = (argc > 1) ? fopen(argv[1], "w") : stdout;
    long xs[64], e1s[64], e2s[64];
    int nx = 0, n1 = 0, n2 = 0, i, j, k;
    long probe[] = { 0, 1, -1, M/4, -M/4, M/2, -M/2, M/2-1, M/2+1,
                     -(M/2-1), -(M/2+1), M, -M, M+1, -M-1,
                     3*M/2, -3*M/2, 2*M, -2*M, 2*M+7, -2*M-7,
                     M-1, -(M-1), 7, -7, 12345, -12345 };
    int np = (int)(sizeof probe / sizeof probe[0]);

    for (i = 0; i < np; i++) { xs[nx++] = probe[i]; e1s[n1++] = probe[i];
                               e2s[n2++] = probe[i]; }

    if (!f) { perror("open"); return 1; }

    for (i = 0; i < nx; i++)
      for (j = 0; j < n1; j++)
        for (k = 0; k < n2; k++) {
            long x = xs[i], e1 = e1s[j], e2 = e2s[k];
            long v, s, e1n;
            int sat = 0, wind = 0;

            v = x + (((3 * e1) - e2) >> 1);
            if (v >= 0) s =  (v + M / 2) >> M_LOG;
            else        s = -((-v + M / 2) >> M_LOG);
            if (s >  1) { s =  1; sat = 1; }
            if (s < -1) { s = -1; sat = 1; }
            e1n = v - s * M;
            if (e1n >  2 * M) { e1n =  2 * M; wind = 1; }
            if (e1n < -2 * M) { e1n = -2 * M; wind = 1; }

            fprintf(f, "%ld %ld %ld %ld %ld %d %d\n",
                    x, e1, e2, s, e1n, sat, wind);
        }

    if (f != stdout) fclose(f);
    return 0;
}
