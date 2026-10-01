// uart_tx.v -- 8N1 UART transmitter.
//
// Pulse `start` high for one clk cycle with `data` valid to begin a
// transmission; `busy` stays high until the stop bit finishes. `data`
// is captured into an internal shift register on `start`, so the
// caller doesn't need to hold it stable for the whole frame.

module uart_tx #(
    parameter CLK_FREQ  = 27_000_000,
    parameter BAUD_RATE = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       start,
    output reg        tx,
    output reg        busy
);
    localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE;

    localparam IDLE = 2'd0, START_BIT = 2'd1, DATA_BITS = 2'd2, STOP_BIT = 2'd3;

    reg [1:0]  state     = IDLE;
    reg [15:0] clk_count = 16'd0;
    reg [2:0]  bit_index = 3'd0;
    reg [7:0]  tx_shift  = 8'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            tx        <= 1'b1; // idle line is high
            busy      <= 1'b0;
            clk_count <= 16'd0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1'b1;
                    if (start) begin
                        tx_shift  <= data;
                        state     <= START_BIT;
                        busy      <= 1'b1;
                        clk_count <= 16'd0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                START_BIT: begin
                    tx <= 1'b0;
                    if (clk_count == (BAUD_DIV - 1)) begin
                        clk_count <= 16'd0;
                        state     <= DATA_BITS;
                        bit_index <= 3'd0;
                    end else begin
                        clk_count <= clk_count + 16'd1;
                    end
                end

                DATA_BITS: begin
                    tx <= tx_shift[bit_index];
                    if (clk_count == (BAUD_DIV - 1)) begin
                        clk_count <= 16'd0;
                        if (bit_index == 3'd7) begin
                            state <= STOP_BIT;
                        end else begin
                            bit_index <= bit_index + 3'd1;
                        end
                    end else begin
                        clk_count <= clk_count + 16'd1;
                    end
                end

                STOP_BIT: begin
                    tx <= 1'b1;
                    if (clk_count == (BAUD_DIV - 1)) begin
                        clk_count <= 16'd0;
                        state     <= IDLE;
                        busy      <= 1'b0;
                    end else begin
                        clk_count <= clk_count + 16'd1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
