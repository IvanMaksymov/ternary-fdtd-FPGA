// ====================================================================
// top_fdtd_hw.v -- synthesis top level.
//
// Its only job is to pull in build/params.vh, which the C golden model
// wrote, and hand those numbers to top_fdtd.  The hardware and the
// reference therefore cannot disagree about NX, M, MC, the source
// frequency or the probe cell: there is exactly one place those values
// are decided, and it is the same place that generated the test
// vectors the design was checked against.
// ====================================================================

`include "build/params.vh"

module top_fdtd_hw (
    input  wire clk,       // 27 MHz onboard oscillator, pin 4
    input  wire uart_rxd,  // pin 70
    output wire uart_txd,  // pin 69
    output wire led0,      // pin 15
    output wire led1       // pin 16
);
    top_fdtd #(
        .NX(`FDTD_NX), .NY(`FDTD_NY),
        .M_LOG(`FDTD_M_LOG), .MC_LOG(`FDTD_MC_LOG), .W(`FDTD_W),
        .PHASE_INC(`FDTD_PHASE_INC),
        .LUT_FILE("build/sine_lut.hex")
    ) u_top (
        .clk(clk), .uart_rxd(uart_rxd), .uart_txd(uart_txd),
        .led0(led0), .led1(led1)
    );
endmodule
