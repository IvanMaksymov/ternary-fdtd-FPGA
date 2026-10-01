// ====================================================================
// tb_fdtd_ternary.v -- differential test of the whole solver.
//
// Runs the RTL core and compares, step by step, the emitted symbol and
// the state accumulator at the probe cell against build/golden.txt,
// which fdtd_golden.c produced with the same parameters, the same
// source LUT file and the same update order.
//
// This is a BIT-EXACT check, not a tolerance check.  The two
// implementations do identical integer arithmetic, so any difference
// at all is a bug in one of them; there is no numerical slack to hide
// in.  A single mismatched step is a failure.
//
// The UART is deliberately not in the loop here: this testbench drives
// probe_ack directly, so a failure points at the solver rather than at
// the link.  tb_top_fdtd.v covers the framing.
// ====================================================================

`timescale 1ns/1ps

`include "build/params.vh"

module tb_fdtd_ternary;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg run = 1'b0;
    always #18.5 clk = ~clk;          // ~27 MHz

    wire signed [1:0]  probe_sym;
    wire signed [15:0] probe_state;
    wire               probe_valid;
    reg                probe_ack = 1'b0;
    wire [31:0]        step_count;
    wire [31:0]        sat_count, wind_count;
    wire               core_idle;

    fdtd_ternary #(
        .NX(`FDTD_NX), .NY(`FDTD_NY),
        .M_LOG(`FDTD_M_LOG), .MC_LOG(`FDTD_MC_LOG), .W(`FDTD_W),
        .PHASE_INC(`FDTD_PHASE_INC),
        .LUT_FILE("build/sine_lut.hex"),
        .PX(`FDTD_PX), .PY(`FDTD_PY)
    ) dut (
        .clk(clk), .rst(rst), .run(run), .diag_src(1'b0),
        .probe_sym(probe_sym), .probe_state(probe_state),
        .probe_valid(probe_valid), .probe_ack(probe_ack),
        .step_count(step_count), .sat_count(sat_count),
        .wind_count(wind_count), .idle(core_idle)
    );

    integer fd, code, nchk, nerr, first_bad;
    integer gstep, gsym, gstate;
    real    gref;

    initial begin
        nchk = 0; nerr = 0; first_bad = -1;

        fd = $fopen("build/golden.txt", "r");
        if (fd == 0) begin
            $display("FAIL: cannot open build/golden.txt");
            $finish;
        end

        repeat (10) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);
        run = 1'b1;

        begin : check_loop
        forever begin
            // wait for the core to present a sample
            while (!probe_valid) @(posedge clk);

            code = $fscanf(fd, "%d %d %d %f\n", gstep, gsym, gstate, gref);
            if (code != 4) disable check_loop;

            if ($signed(probe_sym)   !== gsym ||
                $signed(probe_state) !== gstate) begin
                if (nerr < 20)
                    $display("MISMATCH step %0d: rtl sym=%0d state=%0d | golden sym=%0d state=%0d",
                             gstep, $signed(probe_sym), $signed(probe_state),
                             gsym, gstate);
                if (first_bad < 0) first_bad = gstep;
                nerr = nerr + 1;
            end
            nchk = nchk + 1;

            // acknowledge; the core then starts the next step
            probe_ack <= 1'b1;
            @(posedge clk);
            probe_ack <= 1'b0;
            @(posedge clk);
        end
        end

        run = 1'b0;
        $fclose(fd);

        $display("==============================================");
        $display("steps compared        %0d", nchk);
        $display("mismatches            %0d", nerr);
        $display("saturation events     %0d", sat_count);
        $display("anti-windup clamps    %0d", wind_count);
        if (nerr == 0 && nchk > 0)
            $display("PASS: RTL is bit-exact against the C golden model over %0d steps", nchk);
        else
            $display("FAILED: first divergence at step %0d", first_bad);
        $display("==============================================");
        $finish;
    end

    // safety net: do not let a hung FSM run forever
    initial begin
        #200_000_000;
        $display("FAILED: timeout, core produced %0d of %0d samples",
                 nchk, `FDTD_NSTEP);
        $finish;
    end

endmodule
