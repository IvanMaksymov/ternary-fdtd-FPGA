// uart_rx.v -- 8N1 UART receiver.
//
// Standard start-bit-detect + midpoint-sampling design. Samples each
// bit at its center (not its edge) for noise tolerance: after detecting
// and confirming the start bit at BAUD_DIV/2 into the frame, each
// subsequent sample is exactly one full BAUD_DIV later, which lands at
// that bit's own center given the frame's fixed bit period.
//
// 27MHz / 115200 baud -> BAUD_DIV = 234 (actual baud 115384.6, 0.16%
// error -- well within standard UART tolerance).

module uart_rx #(
    parameter CLK_FREQ  = 27_000_000,
    parameter BAUD_RATE = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid    // pulses high for exactly one clk cycle
);
    localparam integer BAUD_DIV      = CLK_FREQ / BAUD_RATE;
    localparam integer HALF_BAUD_DIV = BAUD_DIV / 2;

    localparam IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    reg [1:0]  state     = IDLE;
    reg [15:0] clk_count = 16'd0;
    reg [2:0]  bit_index = 3'd0;
    reg [7:0]  rx_shift  = 8'd0;

    // 2-flop synchronizer: rx is asynchronous to clk
    reg rx_sync0 = 1'b1, rx_sync1 = 1'b1;
    always @(posedge clk) begin
        rx_sync0 <= rx;
        rx_sync1 <= rx_sync0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            valid     <= 1'b0;
            clk_count <= 16'd0;
            bit_index <= 3'd0;
        end else begin
            valid <= 1'b0; // default; only asserted explicitly below
            case (state)
                IDLE: begin
                    if (rx_sync1 == 1'b0) begin // falling edge = start bit
                        state     <= START;
                        clk_count <= 16'd0;
                    end
                end

                START: begin
                    if (clk_count == HALF_BAUD_DIV[15:0]) begin
                        if (rx_sync1 == 1'b0) begin // still low at midpoint: real start bit
                            state     <= DATA;
                            clk_count <= 16'd0;
                            bit_index <= 3'd0;
                        end else begin
                            state <= IDLE; // false start (noise glitch)
                        end
                    end else begin
                        clk_count <= clk_count + 16'd1;
                    end
                end

                DATA: begin
                    if (clk_count == (BAUD_DIV - 1)) begin
                        clk_count          <= 16'd0;
                        rx_shift[bit_index] <= rx_sync1;
                        if (bit_index == 3'd7) begin
                            state <= STOP;
                        end else begin
                            bit_index <= bit_index + 3'd1;
                        end
                    end else begin
                        clk_count <= clk_count + 16'd1;
                    end
                end

                STOP: begin
                    if (clk_count == (BAUD_DIV - 1)) begin
                        clk_count <= 16'd0;
                        data      <= rx_shift;
                        valid     <= 1'b1;
                        state     <= IDLE;
                    end else begin
                        clk_count <= clk_count + 16'd1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
