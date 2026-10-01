// ====================================================================
// top_fdtd.v -- ternary FDTD core + UART, Tang Nano 20K.
//
// EVERYTHING RUNS IN THE SINGLE 27 MHz OSCILLATOR DOMAIN.  There is no
// PLL and no clock-domain crossing, because there is no video path in
// this target.  That removes a whole class of bugs from the bring-up;
// the HDMI display can be added later, and when it is, the FDTD core
// should stay on this clock and only the display read port crosses.
//
// Host -> FPGA, one command byte:
//     'R'  0x52   reset the core (clears every field and the step count)
//     'G'  0x47   run: step continuously and stream one frame per step
//     'S'  0x53   stop after the current step
//     'D'  0x44   diagnostic: report the SOURCE sample in the state
//                 field instead of the probe state.  The captured
//                 column should then be a clean sine of amplitude
//                 SRC_K*MC repeating every 2^16/PHASE_INC steps.  If it
//                 is constant or zero, the source ROM did not
//                 initialise and nothing downstream means anything.
//     'F'  0x46   back to reporting the field
//
// FPGA -> host, one 5-byte frame per COMPLETED FDTD STEP:
//     [0] 0xA5            sync marker
//     [1] symbol          int8, one of -1, 0, +1   (the emitted symbol)
//     [2] state high      int16 big-endian, the state accumulator a
//     [3] state low         (field value = a * A / M)
//     [4] step low byte   step_count[7:0], increments by 1 every step
//
// The step byte exists because a decimated or duplicated stream is
// otherwise INVISIBLE: it still decodes, still looks like a field, and
// only shows up as a frequency axis that is wrong by an integer factor.
// The host checks that this byte advances by exactly one per frame, so
// a dropped or repeated sample is reported rather than silently
// rescaling the spectrum.
//
// One frame per step, not one per timer tick.  The core stalls until
// the frame has gone out, so the host receives every step in order with
// no decimation.  A free-running transmit timer would silently sample
// every Nth step instead, which aliases the spectrum beyond repair and
// makes the captured stream look like white noise no matter how well
// the solver is working.
//
// At 115200 baud a frame costs 5*10/115200 = 434 us.  One step of a
// 32x32 grid takes 13991 cycles (counted, not estimated -- see
// golden/fsm_check.py) = 518 us at 27 MHz, so the solver is
// compute-bound, the link has 16 % headroom, and the stall in the core
// costs nothing.  A 16384-step run takes about 8.5 s.
// ====================================================================

module top_fdtd #(
    parameter integer NX     = 32,
    parameter integer NY     = 32,
    parameter integer M_LOG  = 13,
    parameter integer MC_LOG = 5,
    parameter integer W      = 18,
    parameter integer PHASE_INC = 16,
    parameter         LUT_FILE  = "build/sine_lut.hex"
) (
    input  wire clk,       // 27 MHz onboard oscillator, pin 4
    input  wire uart_rxd,  // pin 70, from host
    output wire uart_txd,  // pin 69, to host
    output wire led0,      // pin 15, heartbeat
    output wire led1       // pin 16, high while the solver is running
);
    // ----------------------------------------------------------------
    // UART
    // ----------------------------------------------------------------
    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;

    uart_rx #(.CLK_FREQ(27_000_000), .BAUD_RATE(115200)) u_rx (
        .clk(clk), .rst_n(1'b1), .rx(uart_rxd),
        .data(rx_data), .valid(rx_valid)
    );

    uart_tx #(.CLK_FREQ(27_000_000), .BAUD_RATE(115200)) u_tx (
        .clk(clk), .rst_n(1'b1),
        .data(tx_data), .start(tx_start),
        .tx(uart_txd), .busy(tx_busy)
    );

    // ----------------------------------------------------------------
    // command decode
    // ----------------------------------------------------------------
    reg core_rst  = 1'b1;
    reg core_run  = 1'b0;
    reg core_diag = 1'b0;

    always @(posedge clk) begin
        core_rst <= 1'b0;                  // one-shot: pulses for a cycle
        if (rx_valid) begin
            case (rx_data)
                8'h52: begin core_rst <= 1'b1; core_run <= 1'b0; end // 'R'
                8'h47: core_run <= 1'b1;                            // 'G'
                8'h53: core_run  <= 1'b0;                           // 'S'
                8'h44: core_diag <= 1'b1;                           // 'D'
                8'h46: core_diag <= 1'b0;                           // 'F'
                default: ;                 // ignore anything else
            endcase
        end
    end

    // ----------------------------------------------------------------
    // the solver
    // ----------------------------------------------------------------
    wire signed [1:0]  probe_sym;
    wire signed [15:0] probe_state;
    wire               probe_valid;
    reg                probe_ack;
    wire [31:0]        core_step;
    wire               core_idle;

    fdtd_ternary #(
        .NX(NX), .NY(NY), .M_LOG(M_LOG), .MC_LOG(MC_LOG), .W(W),
        .PHASE_INC(PHASE_INC), .LUT_FILE(LUT_FILE)
    ) u_core (
        .clk(clk), .rst(core_rst), .run(core_run), .diag_src(core_diag),
        .probe_sym(probe_sym), .probe_state(probe_state),
        .probe_valid(probe_valid), .probe_ack(probe_ack),
        .step_count(core_step), .sat_count(), .wind_count(), .idle(core_idle)
    );

    // ----------------------------------------------------------------
    // frame sender: 5 bytes, then acknowledge the sample.
    //
    // This state machine has an EXPLICIT RESET and a DEFAULT branch,
    // and both are load-bearing.  tstate is three bits wide with five
    // states, so three encodings are unreachable.  A Verilog initial
    // value covers them in simulation exactly and therefore no
    // testbench can ever exercise them -- but on the FPGA, if the
    // register comes up outside its initialised value, the case
    // matches nothing, the sender stops for ever, probe_ack is never
    // asserted, and the core stalls in S_WAIT after a single step.
    // The symptom is total silence on the wire with the heartbeat LED
    // still blinking, which is indistinguishable from a dead design.
    // ----------------------------------------------------------------
    localparam [2:0] T_IDLE = 3'd0, T_LOAD = 3'd1, T_WAIT = 3'd2,
                     T_DONE = 3'd3, T_ACK  = 3'd4;

    reg [2:0] tstate = T_IDLE;
    reg [2:0] bidx   = 3'd0;
    reg signed [1:0]  sym_l;
    reg signed [15:0] state_l;
    reg [7:0]         step_l;

    always @(*) begin
        case (bidx)
            3'd0: tx_data = 8'hA5;
            3'd1: tx_data = {{6{sym_l[1]}}, sym_l};   // int8 -1/0/+1
            3'd2: tx_data = state_l[15:8];
            3'd3: tx_data = state_l[7:0];
            default: tx_data = step_l;
        endcase
    end

    always @(posedge clk) begin
        if (core_rst) begin
            tstate    <= T_IDLE;
            bidx      <= 3'd0;
            tx_start  <= 1'b0;
            probe_ack <= 1'b0;
        end else begin
        tx_start  <= 1'b0;
        probe_ack <= 1'b0;

        case (tstate)
            T_IDLE: begin
                if (probe_valid) begin
                    sym_l   <= probe_sym;
                    state_l <= probe_state;
                    step_l  <= core_step[7:0];
                    bidx    <= 3'd0;
                    tstate  <= T_LOAD;
                end
            end

            T_LOAD: begin
                if (!tx_busy && !tx_start) begin
                    tx_start <= 1'b1;
                    tstate   <= T_WAIT;
                end
            end

            // wait for busy to RISE then FALL.  Testing only for
            // !busy here would race: uart_tx raises busy one cycle
            // after start, so a bare !busy check reads the stale
            // value and the byte is skipped.
            T_WAIT: begin
                if (tx_busy) tstate <= T_DONE;
            end

            T_DONE: begin
                if (!tx_busy) begin
                    if (bidx == 3'd4) begin
                        probe_ack <= 1'b1;
                        tstate    <= T_ACK;
                    end else begin
                        bidx   <= bidx + 3'd1;
                        tstate <= T_LOAD;
                    end
                end
            end

            // Hold here until the core has actually withdrawn
            // probe_valid.  Returning straight to T_IDLE re-triggers on
            // the SAME sample, because the core does not clear
            // probe_valid until the cycle after it sees probe_ack --
            // which transmits every step exactly twice and halves the
            // apparent sample rate.
            T_ACK: begin
                if (!probe_valid) tstate <= T_IDLE;
            end

            // unreachable in simulation, reachable at power-up
            default: begin
                bidx   <= 3'd0;
                tstate <= T_IDLE;
            end
        endcase
        end
    end

    // ----------------------------------------------------------------
    // LEDs (Tang Nano 20K LEDs are active low)
    // ----------------------------------------------------------------
    localparam integer HALF_PERIOD = 13_499_999;   // 0.5 s at 27 MHz
    reg [23:0] hb_cnt = 24'd0;
    reg        hb     = 1'b0;

    always @(posedge clk) begin
        if (hb_cnt == HALF_PERIOD) begin
            hb_cnt <= 24'd0;
            hb     <= ~hb;
        end else
            hb_cnt <= hb_cnt + 24'd1;
    end

    // led1 blinks once per 256 transmitted frames, about 7.5 Hz while
    // the solver runs.  This is a DIRECT indicator that frames are
    // leaving the chip: a steady led0 with a dark led1 means the core
    // or the sender has stalled, and it says so without any host
    // software, serial port or Octave in the loop.  Debugging the
    // previous stall took a round trip through all three to establish
    // something the board could have reported by itself.
    reg [8:0] frame_cnt = 9'd0;
    always @(posedge clk)
        if (core_rst)       frame_cnt <= 9'd0;
        else if (probe_ack) frame_cnt <= frame_cnt + 9'd1;

    assign led0 = ~hb;
    assign led1 = ~frame_cnt[8];

endmodule
