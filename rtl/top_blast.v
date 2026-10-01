// ====================================================================
// top_blast.v -- BISECTION STAGE 2: does the FRAME SENDER work, with
//                the solver taken out of the loop?
//
// Emits the same 5-byte frame as top_fdtd, with the same state machine,
// the same byte mux and the same probe_valid / probe_ack handshake --
// but driven by a free-running timer instead of by fdtd_ternary. The
// payload is a counter, so the host sees a recognisable ramp.
//
// This splits the remaining possibilities in two:
//
//   blast works, fdtd silent  ->  the SOLVER is not completing steps,
//                                 so probe_valid never rises
//   blast also silent         ->  the SENDER or its handshake is at
//                                 fault, independent of the solver
//
// It is deliberately a copy of the sender rather than a shared module:
// the point is to test the code as it stands in top_fdtd.v, not a
// refactored version of it.
// ====================================================================

module top_blast (
    input  wire clk,       // 27 MHz, pin 4
    input  wire uart_rxd,  // pin 70 (unused, kept for pin compatibility)
    output wire uart_txd,  // pin 69
    output wire led0,      // pin 15
    output wire led1       // pin 16
);
    reg  [7:0] tx_data;
    reg        tx_start = 1'b0;
    wire       tx_busy;

    uart_tx #(.CLK_FREQ(27_000_000), .BAUD_RATE(115200)) u_tx (
        .clk(clk), .rst_n(1'b1),
        .data(tx_data), .start(tx_start),
        .tx(uart_txd), .busy(tx_busy)
    );

    // ----------------------------------------------------------------
    // stand-in for the solver: raise probe_valid every 14000 cycles,
    // which is one 32x32 FDTD step, and drop it on probe_ack
    // ----------------------------------------------------------------
    reg [15:0] tick        = 16'd0;
    reg        probe_valid = 1'b0;
    reg [15:0] counter     = 16'd0;
    reg [31:0] step        = 32'd0;
    reg        probe_ack;

    always @(posedge clk) begin
        if (probe_ack) begin
            probe_valid <= 1'b0;
            tick        <= 16'd0;
        end else if (!probe_valid) begin
            if (tick == 16'd14000) begin
                probe_valid <= 1'b1;
                counter     <= counter + 16'd1;
                step        <= step + 32'd1;
            end else
                tick <= tick + 16'd1;
        end
    end

    // ----------------------------------------------------------------
    // the frame sender, copied verbatim from top_fdtd.v
    // ----------------------------------------------------------------
    localparam [2:0] T_IDLE = 3'd0, T_LOAD = 3'd1, T_WAIT = 3'd2,
                     T_DONE = 3'd3, T_ACK  = 3'd4;

    reg [2:0]  tstate  = T_IDLE;
    reg [2:0]  bidx    = 3'd0;
    reg [15:0] state_l = 16'd0;
    reg [7:0]  step_l  = 8'd0;

    always @(*) begin
        case (bidx)
            3'd0: tx_data = 8'hA5;
            3'd1: tx_data = 8'd1;              // stand-in symbol
            3'd2: tx_data = state_l[15:8];
            3'd3: tx_data = state_l[7:0];
            default: tx_data = step_l;
        endcase
    end

    always @(posedge clk) begin
        tx_start  <= 1'b0;
        probe_ack <= 1'b0;

        case (tstate)
            T_IDLE: if (probe_valid) begin
                        state_l <= counter;
                        step_l  <= step[7:0];
                        bidx    <= 3'd0;
                        tstate  <= T_LOAD;
                    end

            T_LOAD: if (!tx_busy && !tx_start) begin
                        tx_start <= 1'b1;
                        tstate   <= T_WAIT;
                    end

            T_WAIT: if (tx_busy) tstate <= T_DONE;

            T_DONE: if (!tx_busy) begin
                        if (bidx == 3'd4) begin
                            probe_ack <= 1'b1;
                            tstate    <= T_ACK;
                        end else begin
                            bidx   <= bidx + 3'd1;
                            tstate <= T_LOAD;
                        end
                    end

            T_ACK:  if (!probe_valid) tstate <= T_IDLE;

            default: begin bidx <= 3'd0; tstate <= T_IDLE; end
        endcase
    end

    // ----------------------------------------------------------------
    localparam integer HALF_PERIOD = 13_499_999;
    reg [23:0] hb_cnt = 24'd0;
    reg        hb     = 1'b0;
    always @(posedge clk)
        if (hb_cnt == HALF_PERIOD) begin hb_cnt <= 24'd0; hb <= ~hb; end
        else                             hb_cnt <= hb_cnt + 24'd1;

    reg [8:0] frame_cnt = 9'd0;
    always @(posedge clk) if (probe_ack) frame_cnt <= frame_cnt + 9'd1;

    assign led0 = ~hb;
    assign led1 = ~frame_cnt[8];
endmodule
