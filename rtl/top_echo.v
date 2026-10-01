// ====================================================================
// top_echo.v -- BISECTION STAGE 1: is the board, port and flash path
//               working at all?
//
// Echoes every received byte. Nothing else. No solver, no frame sender,
// no ROM. If this does not echo, the fault is outside the FDTD design
// entirely -- the serial port, the bitstream not reaching the device,
// the pin constraints, or the board -- and no amount of staring at the
// solver will help.
//
// led0 is the 1 Hz heartbeat. led1 pulses on each byte echoed, so the
// board reports success without a host even being attached: type
// anything into a terminal and led1 should twitch.
// ====================================================================

module top_echo (
    input  wire clk,       // 27 MHz, pin 4
    input  wire uart_rxd,  // pin 70
    output wire uart_txd,  // pin 69
    output wire led0,      // pin 15
    output wire led1       // pin 16
);
    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data  = 8'd0;
    reg        tx_start = 1'b0;
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

    always @(posedge clk) begin
        tx_start <= 1'b0;
        if (rx_valid && !tx_busy) begin
            tx_data  <= rx_data;
            tx_start <= 1'b1;
        end
    end

    // heartbeat
    localparam integer HALF_PERIOD = 13_499_999;
    reg [23:0] hb_cnt = 24'd0;
    reg        hb     = 1'b0;
    always @(posedge clk)
        if (hb_cnt == HALF_PERIOD) begin hb_cnt <= 24'd0; hb <= ~hb; end
        else                             hb_cnt <= hb_cnt + 24'd1;

    // stretch each echoed byte into a visible pulse
    reg [21:0] act = 22'd0;
    always @(posedge clk)
        if (rx_valid)      act <= 22'h3FFFFF;
        else if (act != 0) act <= act - 22'd1;

    assign led0 = ~hb;
    assign led1 = ~(act != 0);
endmodule
