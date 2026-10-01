// ====================================================================
// tern_encode.v -- the noise-shaped ternary encoder of the paper.
//
//     v       = x + (3*e1 - e2)/2            N(z) = (1-z^-1)(1-z^-1/2)
//     s       = clamp(round(v/M), -1, +1)
//     e1_next = clamp(v - s*M, -2M, +2M)
//     e2_next = e1                           (done by the caller)
//
// This is PURELY COMBINATIONAL; the caller registers the outputs.
//
// NO MULTIPLIERS.  Every arithmetic operation below is an add, a
// subtract, a constant shift or a select:
//
//   3*e1     ->  (e1 <<< 1) + e1
//   /2       ->  arithmetic right shift by 1
//   round/M  ->  add M/2, then arithmetic right shift by M_LOG
//   s*M      ->  a three-way select between +M, 0 and -M
//
// The divide by two is an arithmetic shift, so it floors toward minus
// infinity rather than truncating toward zero.  The C golden model
// uses C's `>>` on a signed value, which gcc also implements as an
// arithmetic shift, so the two agree bit for bit on negative operands.
// This matters: the realised feedback coefficients differ from exactly
// 3/2 and -1/2 by at most one fixed-point unit, and the reference must
// make the SAME one-unit choice or the streams diverge.
//
// W must be wide enough to hold +/-2M (the anti-windup bound) with a
// sign bit, i.e. W >= M_LOG + 3.
// ====================================================================

module tern_encode #(
    parameter integer M_LOG = 13,   // M = 2^M_LOG
    parameter integer W     = 18    // width of the state / error words
) (
    input  wire signed [W-1:0] x,        // state accumulator, this cell
    input  wire signed [W-1:0] e1,       // encoder register 1
    input  wire signed [W-1:0] e2,       // encoder register 2
    output wire signed [1:0]   s,        // emitted symbol, -1 / 0 / +1
    output wire signed [W-1:0] e1_next,  // new e1 (e2_next = e1, caller)
    output wire                sat,      // |round(v/M)| > 1 this call
    output wire                wind      // anti-windup clamp fired
);
    // Internal width: room for v = x + 2*e1 + ... without wrapping.
    localparam integer WI = W + 3;

    localparam signed [WI-1:0] MVAL   =  (1 <<< M_LOG);
    localparam signed [WI-1:0] MHALF  =  (1 <<< (M_LOG-1));
    localparam signed [WI-1:0] WINDHI =  (1 <<< (M_LOG+1));   // +2M
    localparam signed [WI-1:0] WINDLO = -(1 <<< (M_LOG+1));   // -2M

    wire signed [WI-1:0] xe  = {{(WI-W){x [W-1]}}, x };
    wire signed [WI-1:0] e1e = {{(WI-W){e1[W-1]}}, e1};
    wire signed [WI-1:0] e2e = {{(WI-W){e2[W-1]}}, e2};

    // (3*e1 - e2) >>> 1, with 3*e1 written as a shift plus an add
    wire signed [WI-1:0] three_e1 = (e1e <<< 1) + e1e;
    wire signed [WI-1:0] fb       = (three_e1 - e2e) >>> 1;

    wire signed [WI-1:0] v = xe + fb;

    // round-to-nearest divide by M, done on a non-negative operand in
    // both branches so the shift is an exact floor division
    wire signed [WI-1:0] vmag = v[WI-1] ? -v : v;
    wire signed [WI-1:0] qmag = (vmag + MHALF) >>> M_LOG;
    wire signed [WI-1:0] q    = v[WI-1] ? -qmag : qmag;

    assign sat = (q > 1) || (q < -1);

    assign s = (q >  1) ?  2'sb01 :
               (q < -1) ?  2'sb11 : q[1:0];

    // v - s*M, as a three-way select
    wire signed [WI-1:0] sM     = (s == 2'sb01) ?  MVAL :
                                  (s == 2'sb11) ? -MVAL : {WI{1'b0}};
    wire signed [WI-1:0] e1_raw = v - sM;

    assign wind = (e1_raw > WINDHI) || (e1_raw < WINDLO);

    wire signed [WI-1:0] e1_clamped = (e1_raw > WINDHI) ? WINDHI :
                                      (e1_raw < WINDLO) ? WINDLO : e1_raw;

    assign e1_next = e1_clamped[W-1:0];

endmodule
