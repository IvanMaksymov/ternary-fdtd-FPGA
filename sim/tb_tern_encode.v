// ====================================================================
// tb_tern_encode.v -- unit test: the Verilog encoder against the C one.
//
// Replays build/enc_vectors.txt, which enc_vectors.c generated from the
// same arithmetic the golden FDTD model uses, and checks s, e1_next,
// sat and wind on every line.  Any mismatch is printed with the input
// that produced it.
//
// This isolates the encoder from the solver: if the full-core test
// fails and this one passes, the fault is in the FSM or the memories,
// not in the quantiser.
// ====================================================================

`timescale 1ns/1ps

module tb_tern_encode;

    localparam integer M_LOG = 13;
    localparam integer W     = 18;

    reg  signed [W-1:0] x, e1, e2;
    wire signed [1:0]   s;
    wire signed [W-1:0] e1_next;
    wire                sat, wind;

    tern_encode #(.M_LOG(M_LOG), .W(W)) dut
        (.x(x), .e1(e1), .e2(e2), .s(s), .e1_next(e1_next),
         .sat(sat), .wind(wind));

    integer fd, code, nline, nerr;
    integer gx, ge1, ge2, gs, ge1n, gsat, gwind;

    initial begin
        nline = 0;
        nerr  = 0;

        fd = $fopen("build/enc_vectors.txt", "r");
        if (fd == 0) begin
            $display("FAIL: cannot open build/enc_vectors.txt");
            $finish;
        end

        begin : read_loop
        forever begin
            code = $fscanf(fd, "%d %d %d %d %d %d %d\n",
                           gx, ge1, ge2, gs, ge1n, gsat, gwind);
            if (code != 7) disable read_loop;

            x  = gx;
            e1 = ge1;
            e2 = ge2;
            #1;

            if ($signed(s)       !== gs   ||
                $signed(e1_next) !== ge1n ||
                sat              !== gsat[0] ||
                wind             !== gwind[0]) begin
                if (nerr < 20)
                    $display("MISMATCH line %0d: x=%0d e1=%0d e2=%0d | rtl s=%0d e1n=%0d sat=%0b wind=%0b | c s=%0d e1n=%0d sat=%0b wind=%0b",
                             nline, gx, ge1, ge2,
                             $signed(s), $signed(e1_next), sat, wind,
                             gs, ge1n, gsat[0], gwind[0]);
                nerr = nerr + 1;
            end
            nline = nline + 1;
        end
        end

        $fclose(fd);

        $display("----------------------------------------------");
        $display("tern_encode: %0d vectors, %0d mismatches", nline, nerr);
        if (nerr == 0 && nline > 0)
            $display("PASS: encoder is bit-exact against the C model");
        else
            $display("FAILED");
        $display("----------------------------------------------");
        $finish;
    end

endmodule
