// ====================================================================
// tb_top_fdtd.v -- framing test.  THIS IS THE ONE THAT WAS MISSING.
//
// The solver can be bit-exact and the capture still be wrong, because
// a duplicated or dropped frame produces a stream that decodes
// perfectly and simply reports the wrong sample rate.  That failure is
// invisible in the data: the field looks plausible and only the
// frequency axis is wrong, by an integer factor.  It is exactly what
// went wrong on the first hardware run -- every step was transmitted
// twice, because the frame sender returned to its idle state in the
// same cycle it acknowledged the sample, before the core had withdrawn
// probe_valid, and so re-triggered on the sample it had just sent.
//
// So this testbench decodes the serial line the way the host does and
// asserts the two properties that matter:
//
//   1. exactly one frame per FDTD step, no duplicates, no drops
//   2. the step byte advances by exactly one every frame
//   3. the payload carries the value it is supposed to
//
// The third is why it runs in diagnostic mode ('D'): the field at the
// probe is still zero this early, so a frame of zeros would pass a
// payload check that was doing nothing.  In diagnostic mode the frame
// carries the source sample, which is non-zero from the start and is
// compared against the same hex file the hardware ROM is loaded from.
// That exercises the whole path -- ROM initialisation, the 16-bit
// narrowing, the byte order -- on values that are not all zero.
//
// It sends 'G' over the receive pin first, so the command path is
// covered as well.
// ====================================================================

`timescale 1ns/1ps

`include "build/params.vh"

module tb_top_fdtd;

    localparam integer CLK_FREQ  = 27_000_000;
    localparam integer BAUD_RATE = 115200;
    localparam integer BAUD_DIV  = CLK_FREQ / BAUD_RATE;
    localparam integer NFRAME    = 24;      // frames to check

    reg clk = 1'b0;
    always #18.5 clk = ~clk;                // ~27 MHz

    reg  host_tx = 1'b1;                    // host -> FPGA line
    wire fpga_tx;                           // FPGA -> host line
    wire led0, led1;

    top_fdtd #(
        .NX(`FDTD_NX), .NY(`FDTD_NY),
        .M_LOG(`FDTD_M_LOG), .MC_LOG(`FDTD_MC_LOG), .W(`FDTD_W),
        .PHASE_INC(`FDTD_PHASE_INC),
        .LUT_FILE("build/sine_lut.hex")
    ) dut (
        .clk(clk), .uart_rxd(host_tx), .uart_txd(fpga_tx),
        .led0(led0), .led1(led1)
    );

    // ----------------------------------------------------------------
    // host transmitter: shift one byte out at the baud rate
    // ----------------------------------------------------------------
    task host_send(input [7:0] b);
        integer i;
        begin
            host_tx = 1'b0;                          // start bit
            repeat (BAUD_DIV) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                host_tx = b[i];
                repeat (BAUD_DIV) @(posedge clk);
            end
            host_tx = 1'b1;                          // stop bit
            repeat (BAUD_DIV) @(posedge clk);
        end
    endtask

    // ----------------------------------------------------------------
    // host receiver: start-bit detect, then sample at each bit centre
    // ----------------------------------------------------------------
    task host_recv(output [7:0] b);
        integer i;
        begin
            @(negedge fpga_tx);                      // start bit
            repeat (BAUD_DIV + BAUD_DIV/2) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = fpga_tx;
                if (i < 7) repeat (BAUD_DIV) @(posedge clk);
            end
        end
    endtask

    // the same file the design's ROM is initialised from
    localparam integer LUT_N = (1 << 6);
    reg signed [15:0] ref_lut [0:LUT_N-1];
    initial $readmemh("build/sine_lut.hex", ref_lut);

    integer nerr, nframe, i;
    reg [7:0] b0, b1, b2, b3, b4;
    integer   prev_step;
    integer   sym, state;

    initial begin
        nerr = 0; nframe = 0; prev_step = -1;

        repeat (200) @(posedge clk);
        host_send(8'h52);                            // 'R' reset
        repeat (BAUD_DIV*4) @(posedge clk);
        host_send(8'h44);                            // 'D' diagnostic
        repeat (BAUD_DIV*4) @(posedge clk);
        host_send(8'h47);                            // 'G' run

        $display("----------------------------------------------");
        $display("decoding %0d frames from the serial line", NFRAME);

        while (nframe < NFRAME) begin
            host_recv(b0);
            if (b0 !== 8'hA5) begin
                $display("FAIL: expected sync 0xA5, got 0x%02h", b0);
                nerr = nerr + 1;
            end
            host_recv(b1);
            host_recv(b2);
            host_recv(b3);
            host_recv(b4);

            sym   = $signed(b1);
            state = $signed({b2, b3});

            if (prev_step >= 0) begin
                if (b4 !== ((prev_step + 1) % 256)) begin
                    $display("FAIL: step byte jumped %0d -> %0d (expected %0d).  %s",
                             prev_step, b4, (prev_step+1)%256,
                             (b4 == prev_step) ? "SAME STEP SENT TWICE"
                                               : "sample dropped");
                    nerr = nerr + 1;
                end
            end
            // In diagnostic mode the payload is the source sample used
            // during this step.  The step byte is the count AFTER the
            // step completed, so the sample belongs to step b4-1:
            //     LUT index = ((b4-1) * PHASE_INC) >> (PHASE_BITS - LUT_LOG)
            begin : payload_check
                integer idx, expect;
                idx    = (((b4 - 1) * `FDTD_PHASE_INC) >> 10) % LUT_N;
                expect = $signed(ref_lut[idx]);
                if (state !== expect) begin
                    $display("FAIL: step %0d payload %0d, expected LUT[%0d] = %0d",
                             b4, state, idx, expect);
                    nerr = nerr + 1;
                end
            end
            $display("  frame %2d: step %3d  sym %+0d  src %0d",
                     nframe, b4, sym, state);
            prev_step = b4;
            nframe = nframe + 1;
        end

        $display("----------------------------------------------");
        $display("frames decoded        %0d", nframe);
        $display("framing errors        %0d", nerr);
        if (nerr == 0)
            $display("PASS: one frame per step, step byte contiguous, source ROM correct");
        else
            $display("FAILED");
        $display("----------------------------------------------");
        $finish;
    end

    initial begin
        #500_000_000;
        $display("FAILED: timeout after %0d frames", nframe);
        $finish;
    end

endmodule
