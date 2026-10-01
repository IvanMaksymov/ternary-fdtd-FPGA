// ====================================================================
// fdtd_ternary.v -- 2D TMz ternary FDTD core, as published.
//
// THE POINT OF THIS MODULE, in one paragraph.  The state accumulator
// and the quantiser are SEPARATE.  Each cell holds an integer state
// `a` representing the field value a*A/M, and that state is never
// drained: it accumulates the discrete curl of the neighbouring
// SYMBOLS for all time, exactly as the Yee update accumulates the
// curl of the neighbouring fields.  What the cell publishes to its
// neighbours is a two-bit symbol s in {-1,0,+1} produced from the
// state by a noise-shaped encoder with its own two registers e1, e2.
// An earlier design subtracted the quantiser threshold back out of
// the accumulator on every firing; that removes the time integration
// and is not this algorithm.
//
// Fixed-point convention:
//     symbol s in {-1,0,+1}   represents the field value   s*A
//     state  a (signed W)     represents the field value   a*A/M
//     MC = M*S must be an exact integer;  here both are powers of two
//
// Update, per cell, per half-step:
//     aHx += MC * ( sEz[i,j]   - sEz[i,j+1] )
//     aHy += MC * ( sEz[i+1,j] - sEz[i,j]   )
//     aEz += MC * ( (sHy[i,j] - sHy[i-1,j]) - (sHx[i,j] - sHx[i,j-1]) )
// followed by s = tern_encode(a, e1, e2) for each component.
//
// Because s is in {-1,0,+1} the differences are in [-2,+2], so
// MC*(difference) is a constant shift of a three-valued quantity: a
// shift and a select, never a multiply.  Together with tern_encode's
// multiplier-free datapath, the whole update compiles with no
// multiply instruction and no DSP block (build with yosys -nodsp to
// confirm).
//
// PEC walls: the H sweep covers 0 <= i,j <= N-2 and the E sweep
// 1 <= i,j <= N-2, which holds Ez at zero on the boundary.
// ====================================================================

module fdtd_ternary #(
    parameter integer NX     = 32,
    parameter integer NY     = 32,
    parameter integer M_LOG  = 13,   // M  = 2^M_LOG
    parameter integer MC_LOG = 5,    // MC = 2^MC_LOG, so S = 2^(MC_LOG-M_LOG)
    parameter integer W      = 18,   // state / error word width, signed
    parameter integer PHASE_BITS = 16,
    parameter integer PHASE_INC  = 16,   // see the golden model for the rule
    parameter integer LUT_LOG    = 6,
    parameter         LUT_FILE   = "build/sine_lut.hex",
    parameter integer SX = NX/2, SY = NY/2,   // source cell
    parameter integer PX = NX/4, PY = NY/4    // probe cell
) (
    input  wire               clk,
    input  wire               rst,        // synchronous, active high

    input  wire               run,        // level: high = keep stepping
    input  wire               diag_src,   // 1 = report src_val instead of
                                          //     the probe state

    // one sample per completed FDTD step, at the probe cell
    output reg  signed [1:0]  probe_sym,
    output reg  signed [15:0] probe_state,
    output reg                probe_valid, // one-cycle pulse
    input  wire               probe_ack,   // consumer took it; core resumes

    output wire [31:0]        step_count,
    output wire [31:0]        sat_count,
    output wire [31:0]        wind_count,
    output wire               idle
);
    localparam integer SIZE = NX * NY;
    localparam integer AW   = $clog2(SIZE);
    localparam integer LUT_N = (1 << LUT_LOG);

    // ----------------------------------------------------------------
    // field memories
    // ----------------------------------------------------------------
    (* ram_style = "block" *) reg signed [1:0] sEz_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [1:0] sHx_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [1:0] sHy_mem [0:SIZE-1];

    (* ram_style = "block" *) reg signed [W-1:0] aEz_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [W-1:0] aHx_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [W-1:0] aHy_mem [0:SIZE-1];

    (* ram_style = "block" *) reg signed [W-1:0] e1Ez_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [W-1:0] e2Ez_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [W-1:0] e1Hx_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [W-1:0] e2Hx_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [W-1:0] e1Hy_mem [0:SIZE-1];
    (* ram_style = "block" *) reg signed [W-1:0] e2Hy_mem [0:SIZE-1];

    // ----------------------------------------------------------------
    // source LUT.  Loaded from a file the golden model writes, so the
    // hardware and the reference drive the grid with identical integers.
    // ----------------------------------------------------------------
    reg signed [15:0] sine_lut [0:LUT_N-1];

    // The path is a STRING LITERAL, not the LUT_FILE parameter.
    // iverilog resolves a parameter used as a $readmemh argument
    // without complaint, but yosys does not reliably constant-fold a
    // parameter passed down through a module hierarchy into a system
    // task, and when it fails the ROM is simply left uninitialised --
    // silently, with no warning, and only on hardware.  A source stuck
    // at a constant then drives the field in one direction for ever,
    // the encoder never has reason to emit -1, and the state climbs
    // until it pins.  Both the simulation and the synthesis flows
    // write this same path, so a literal costs nothing.
    initial $readmemh("build/sine_lut.hex", sine_lut);

    reg [PHASE_BITS-1:0] phase;
    wire [LUT_LOG-1:0]   lut_idx = phase[PHASE_BITS-1 -: LUT_LOG];
    wire signed [15:0]   src_val = sine_lut[lut_idx];

    // ----------------------------------------------------------------
    // read ports: registered address, registered data (BRAM friendly).
    // Data for an address set in cycle n is valid in cycle n+2.
    // ----------------------------------------------------------------
    reg [AW-1:0] ra_sEz, ra_sHx, ra_sHy, ra_aEz, ra_aHx, ra_aHy;
    reg [AW-1:0] ra_e1Ez, ra_e2Ez, ra_e1Hx, ra_e2Hx, ra_e1Hy, ra_e2Hy;

    reg signed [1:0]   q_sEz, q_sHx, q_sHy;
    reg signed [W-1:0] q_aEz, q_aHx, q_aHy;
    reg signed [W-1:0] q_e1Ez, q_e2Ez, q_e1Hx, q_e2Hx, q_e1Hy, q_e2Hy;

    always @(posedge clk) begin
        q_sEz  <= sEz_mem [ra_sEz ];
        q_sHx  <= sHx_mem [ra_sHx ];
        q_sHy  <= sHy_mem [ra_sHy ];
        q_aEz  <= aEz_mem [ra_aEz ];
        q_aHx  <= aHx_mem [ra_aHx ];
        q_aHy  <= aHy_mem [ra_aHy ];
        q_e1Ez <= e1Ez_mem[ra_e1Ez];
        q_e2Ez <= e2Ez_mem[ra_e2Ez];
        q_e1Hx <= e1Hx_mem[ra_e1Hx];
        q_e2Hx <= e2Hx_mem[ra_e2Hx];
        q_e1Hy <= e1Hy_mem[ra_e1Hy];
        q_e2Hy <= e2Hy_mem[ra_e2Hy];
    end

    // ----------------------------------------------------------------
    // write port
    // ----------------------------------------------------------------
    reg [AW-1:0]       wa;
    reg                we_h, we_e, we_clear;
    reg signed [1:0]   wd_sHx, wd_sHy, wd_sEz;
    reg signed [W-1:0] wd_aHx, wd_aHy, wd_aEz;
    reg signed [W-1:0] wd_e1Hx, wd_e2Hx, wd_e1Hy, wd_e2Hy, wd_e1Ez, wd_e2Ez;

    always @(posedge clk) begin
        if (we_clear) begin
            sEz_mem[wa] <= 2'sd0;  sHx_mem[wa] <= 2'sd0;  sHy_mem[wa] <= 2'sd0;
            aEz_mem[wa] <= {W{1'b0}}; aHx_mem[wa] <= {W{1'b0}}; aHy_mem[wa] <= {W{1'b0}};
            e1Ez_mem[wa] <= {W{1'b0}}; e2Ez_mem[wa] <= {W{1'b0}};
            e1Hx_mem[wa] <= {W{1'b0}}; e2Hx_mem[wa] <= {W{1'b0}};
            e1Hy_mem[wa] <= {W{1'b0}}; e2Hy_mem[wa] <= {W{1'b0}};
        end else begin
            if (we_h) begin
                sHx_mem [wa] <= wd_sHx;   sHy_mem [wa] <= wd_sHy;
                aHx_mem [wa] <= wd_aHx;   aHy_mem [wa] <= wd_aHy;
                e1Hx_mem[wa] <= wd_e1Hx;  e2Hx_mem[wa] <= wd_e2Hx;
                e1Hy_mem[wa] <= wd_e1Hy;  e2Hy_mem[wa] <= wd_e2Hy;
            end
            if (we_e) begin
                sEz_mem [wa] <= wd_sEz;
                aEz_mem [wa] <= wd_aEz;
                e1Ez_mem[wa] <= wd_e1Ez;  e2Ez_mem[wa] <= wd_e2Ez;
            end
        end
    end

    // ----------------------------------------------------------------
    // addresses
    // ----------------------------------------------------------------
    localparam integer XW = $clog2(NX);
    localparam integer YW = $clog2(NY);

    reg [XW-1:0] x;
    reg [YW-1:0] y;

    // Address = y*NX + x.  NX and NY MUST be powers of two, so this is
    // a bit concatenation rather than a multiply -- written out as one
    // so that no multiplier can be inferred here and quietly falsify
    // the multiplier-free claim the design is meant to demonstrate.
    // The neighbour forms never carry across the x field: the H sweep
    // keeps x <= NX-2 so x+1 fits, and the E sweep keeps x >= 1.
    wire [AW-1:0] a_cur   = {y,           x          };
    wire [AW-1:0] a_up    = {y + 1'b1,    x          };
    wire [AW-1:0] a_right = {y,           x + 1'b1   };
    wire [AW-1:0] a_left  = {y,           x - 1'b1   };
    wire [AW-1:0] a_down  = {y - 1'b1,    x          };

    // ----------------------------------------------------------------
    // pipeline registers
    // ----------------------------------------------------------------
    reg signed [1:0]   ez_cur, ez_up, ez_r;
    reg signed [1:0]   hx_cur, hx_d, hy_cur, hy_l;
    reg signed [W-1:0] ahx, ahy, aez;
    reg signed [W-1:0] e1hx, e2hx, e1hy, e2hy, e1ez, e2ez;
    reg signed [W-1:0] ax, ay, ae;

    // MC * difference, as a shift of a sign-extended small signed word.
    // d spans [-4,+4] for the E curl and [-2,+2] for the H curls, so 5
    // bits signed is enough; the shift is by a constant, never a multiply.
    function signed [W-1:0] mcshift(input signed [4:0] d);
        mcshift = $signed({{(W-5){d[4]}}, d}) <<< MC_LOG;
    endfunction

    // sign-extend a two-bit symbol to 5 bits signed, explicitly.  Bare
    // concatenations are UNSIGNED in Verilog, so this must not be left
    // to {{n{s[1]}}, s} inside an arithmetic expression.
    function signed [4:0] sx(input signed [1:0] t);
        sx = {{3{t[1]}}, t};
    endfunction

    // ----------------------------------------------------------------
    // encoders (combinational)
    // ----------------------------------------------------------------
    wire signed [1:0]   s_hx, s_hy, s_ez;
    wire signed [W-1:0] e1n_hx, e1n_hy, e1n_ez;
    wire sat_hx, sat_hy, sat_ez, wnd_hx, wnd_hy, wnd_ez;

    tern_encode #(.M_LOG(M_LOG), .W(W)) enc_hx
        (.x(ax), .e1(e1hx), .e2(e2hx), .s(s_hx), .e1_next(e1n_hx),
         .sat(sat_hx), .wind(wnd_hx));
    tern_encode #(.M_LOG(M_LOG), .W(W)) enc_hy
        (.x(ay), .e1(e1hy), .e2(e2hy), .s(s_hy), .e1_next(e1n_hy),
         .sat(sat_hy), .wind(wnd_hy));
    tern_encode #(.M_LOG(M_LOG), .W(W)) enc_ez
        (.x(ae), .e1(e1ez), .e2(e2ez), .s(s_ez), .e1_next(e1n_ez),
         .sat(sat_ez), .wind(wnd_ez));

    // ----------------------------------------------------------------
    // FSM
    // ----------------------------------------------------------------
    localparam [4:0]
        S_CLEAR = 5'd0,  S_IDLE = 5'd1,
        S_HA = 5'd2, S_HB = 5'd3, S_HC = 5'd4, S_HD = 5'd5,
        S_HE = 5'd6, S_HF = 5'd7, S_HW = 5'd8, S_HN = 5'd9,
        S_EA = 5'd10, S_EB = 5'd11, S_EC = 5'd12, S_ED = 5'd13,
        S_EE = 5'd14, S_EW = 5'd15, S_EN = 5'd16,
        S_EMIT = 5'd17, S_WAIT = 5'd18;

    reg [4:0]  state;
    reg [AW:0] clear_idx;
    reg [31:0] nstep;
    reg [31:0] nsat, nwind;

    assign step_count = nstep;
    assign sat_count  = nsat;
    assign wind_count = nwind;
    assign idle       = (state == S_IDLE);

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_CLEAR;
            clear_idx   <= 0;
            x <= 0; y <= 0;
            phase       <= 0;
            nstep       <= 0;
            nsat        <= 0;
            nwind       <= 0;
            we_clear    <= 1'b1;
            we_h        <= 1'b0;
            we_e        <= 1'b0;
            probe_valid <= 1'b0;
            wa          <= {AW{1'b0}};
            // read addresses are reset so the read registers never
            // latch from an X index during the clear sweep
            ra_sEz  <= {AW{1'b0}};  ra_sHx  <= {AW{1'b0}};
            ra_sHy  <= {AW{1'b0}};  ra_aEz  <= {AW{1'b0}};
            ra_aHx  <= {AW{1'b0}};  ra_aHy  <= {AW{1'b0}};
            ra_e1Ez <= {AW{1'b0}};  ra_e2Ez <= {AW{1'b0}};
            ra_e1Hx <= {AW{1'b0}};  ra_e2Hx <= {AW{1'b0}};
            ra_e1Hy <= {AW{1'b0}};  ra_e2Hy <= {AW{1'b0}};
        end else begin
            we_h <= 1'b0;
            we_e <= 1'b0;
            we_clear <= 1'b0;

            case (state)

            // ---------------- clear every array ----------------
            S_CLEAR: begin
                wa       <= clear_idx[AW-1:0];
                we_clear <= 1'b1;
                if (clear_idx == SIZE-1) begin
                    clear_idx <= 0;
                    state     <= S_IDLE;
                end else
                    clear_idx <= clear_idx + 1'b1;
            end

            S_IDLE: begin
                if (run) begin
                    x <= 0; y <= 0;
                    state <= S_HA;
                end
            end

            // ================= H sweep =================
            S_HA: begin
                ra_sEz  <= a_cur;
                ra_aHx  <= a_cur;  ra_aHy  <= a_cur;
                ra_e1Hx <= a_cur;  ra_e2Hx <= a_cur;
                ra_e1Hy <= a_cur;  ra_e2Hy <= a_cur;
                state   <= S_HB;
            end

            S_HB: begin
                ra_sEz <= a_up;         // data for a_cur lands next cycle
                state  <= S_HC;
            end

            S_HC: begin
                ez_cur <= q_sEz;
                ahx    <= q_aHx;   ahy  <= q_aHy;
                e1hx   <= q_e1Hx;  e2hx <= q_e2Hx;
                e1hy   <= q_e1Hy;  e2hy <= q_e2Hy;
                ra_sEz <= a_right;
                state  <= S_HD;
            end

            S_HD: begin ez_up <= q_sEz; state <= S_HE; end
            S_HE: begin ez_r  <= q_sEz; state <= S_HF; end

            S_HF: begin
                // Hx += S*(Ez - Ez_up);  Hy += S*(Ez_r - Ez)
                ax    <= ahx + mcshift(sx(ez_cur) - sx(ez_up ));
                ay    <= ahy + mcshift(sx(ez_r  ) - sx(ez_cur));
                state <= S_HW;
            end

            S_HW: begin
                wa      <= a_cur;
                wd_sHx  <= s_hx;    wd_sHy  <= s_hy;
                wd_aHx  <= ax;      wd_aHy  <= ay;
                wd_e1Hx <= e1n_hx;  wd_e2Hx <= e1hx;   // e2 <- old e1
                wd_e1Hy <= e1n_hy;  wd_e2Hy <= e1hy;
                we_h    <= 1'b1;
                nsat    <= nsat  + sat_hx + sat_hy;
                nwind   <= nwind + wnd_hx + wnd_hy;
                state   <= S_HN;
            end

            S_HN: begin
                if (x == NX-2) begin
                    if (y == NY-2) begin
                        x <= 1; y <= 1;
                        state <= S_EA;          // H sweep done
                    end else begin
                        x <= 0; y <= y + 1'b1;
                        state <= S_HA;
                    end
                end else begin
                    x     <= x + 1'b1;
                    state <= S_HA;
                end
            end

            // ================= E sweep =================
            S_EA: begin
                ra_sHx  <= a_cur;  ra_sHy  <= a_cur;
                ra_aEz  <= a_cur;
                ra_e1Ez <= a_cur;  ra_e2Ez <= a_cur;
                state   <= S_EB;
            end

            S_EB: begin
                ra_sHx <= a_down;
                ra_sHy <= a_left;
                state  <= S_EC;
            end

            S_EC: begin
                hx_cur <= q_sHx;   hy_cur <= q_sHy;
                aez    <= q_aEz;
                e1ez   <= q_e1Ez;  e2ez   <= q_e2Ez;
                state  <= S_ED;
            end

            S_ED: begin
                hx_d  <= q_sHx;
                hy_l  <= q_sHy;
                state <= S_EE;
            end

            S_EE: begin
                // Ez += S*((Hy - Hy_l) - (Hx - Hx_d)), plus the source
                ae <= aez
                    + mcshift( (sx(hy_cur) - sx(hy_l))
                             - (sx(hx_cur) - sx(hx_d)) )
                    // $signed() is not decoration: a bare concatenation
                    // is UNSIGNED, and one unsigned operand makes the
                    // whole addition unsigned in Verilog.
                    + (((x == SX) && (y == SY))
                         ? $signed({{(W-16){src_val[15]}}, src_val})
                         : $signed({W{1'b0}}));
                state <= S_EW;
            end

            S_EW: begin
                wa      <= a_cur;
                wd_sEz  <= s_ez;
                wd_aEz  <= ae;
                wd_e1Ez <= e1n_ez;  wd_e2Ez <= e1ez;
                we_e    <= 1'b1;
                nsat    <= nsat  + sat_ez;
                nwind   <= nwind + wnd_ez;
                if ((x == PX) && (y == PY)) begin
                    probe_sym   <= s_ez;
                    // |a| stays far below 2^15, so the narrowing is exact.
                    // In diagnostic mode the frame carries the source
                    // sample instead, which is the only way to see from
                    // the host whether the ROM initialised on hardware.
                    probe_state <= diag_src ? src_val : ae[15:0];
                end
                state <= S_EN;
            end

            S_EN: begin
                if (x == NX-2) begin
                    if (y == NY-2)  state <= S_EMIT;
                    else begin x <= 1; y <= y + 1'b1; state <= S_EA; end
                end else begin
                    x     <= x + 1'b1;
                    state <= S_EA;
                end
            end

            // ---------------- end of step ----------------
            S_EMIT: begin
                phase       <= phase + PHASE_INC[PHASE_BITS-1:0];
                nstep       <= nstep + 1'b1;
                probe_valid <= 1'b1;
                state       <= S_WAIT;
            end

            // Stall until the sample has actually been taken.  This is
            // what keeps one transmitted sample equal to one FDTD step:
            // a free-running timer instead would decimate the stream and
            // alias the spectrum.
            S_WAIT: begin
                if (probe_ack) begin
                    probe_valid <= 1'b0;
                    if (run) begin x <= 0; y <= 0; state <= S_HA; end
                    else           state <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
