% ====================================================================
% fdtd_capture.m -- capture the ternary FDTD probe stream from the
%                   Tang Nano 20K and analyse it.
%
% THE SAMPLING RATE.  One received frame is one completed FDTD step,
% because the solver stalls until its frame has been transmitted.  So
% the sample interval in PHYSICAL time is exactly the Courant number
% S = 2^(MC_LOG - M_LOG), and the frequency axis below is in physical
% units, not in Hz of wall clock.  This is the point that has to be got
% right: a transmitter on a free-running timer would send every Nth
% step instead, and the spectrum of that decimated stream is aliased
% beyond recovery however well the solver is running.
%
% Frame format, 4 bytes:
%     [0] 0xA5   sync
%     [1] symbol as int8, one of -1, 0, +1
%     [2] state accumulator, int16 big-endian, high byte
%     [3]                                      low byte
%     [4] step_count[7:0], which MUST advance by exactly one per frame
%
% Commands to the board: 'R' reset, 'G' run, 'S' stop.
%
% Requires the instrument-control package:
%     pkg install -forge instrument-control
% ====================================================================

clear; close all;

% safe_close lives in host/safe_close.m: a script-local function goes
% out of scope before onCleanup runs, leaving the port open.
if exist("host", "dir"), addpath("host"); end
pkg load instrument-control;

% ----------------------------------------------------------------
% configuration -- these must match the Makefile's HW_* values
% ----------------------------------------------------------------

SERIAL_PORT = "/dev/serial/by-id/usb-SIPEED_USB_Debugger_2025030317-if01-port0";
BAUD_RATE   = 115200;

% Parameters come from build/params.vh, written by the golden model,
% so the host cannot disagree with the bitstream about S.
PAR     = read_params("build/params.vh");
M_LOG   = PAR.M_LOG;
MC_LOG  = PAR.MC_LOG;
WAVELEN = PAR.WAVELEN;
% The wave needs |probe-source|/S steps to reach the probe (2637 at
% S = 1/256), and the drive line lands in bin (N*S/WAVELEN), so the
% record must hold the transient PLUS enough periods to resolve it.
% 40 periods after arrival is a reasonable default; at 518 us per step
% this run takes about 2.5 minutes.
% Frames to collect: taken from params.vh, so the capture and the
% reference run that will be compared against it are the same length.
NSAMP   = PAR.NSTEP;

% Diagnostic mode.  Set true and the board reports the SOURCE sample in
% the state field instead of the probe field.  The captured column must
% then be a clean sine of amplitude SRC_K*MC repeating every
% 2^16/PHASE_INC steps.  If it is constant or zero, the source ROM did
% not initialise in the bitstream and nothing downstream means anything
% -- a constant source drives the field one way for ever, so the encoder
% never emits -1 and the state climbs until it pins.  Run this ONCE
% after any change to the build before trusting a field capture.
% Set by the environment, so it survives the `clear` above and can be
% switched on from the Makefile without editing this file:
%     make diag        diagnostic run
%     make capture     normal field capture
DIAG_SOURCE = ~isempty(getenv("FDTD_DIAG"));
SRC_K       = 3;

S = 2^(MC_LOG - M_LOG);          % Courant number = physical time per step
M = 2^M_LOG;

% The diagnostic only has to show the source oscillating, and the source
% period is 2^16/PHASE_INC steps, so two periods is ample.  No point
% spending two and a half minutes on it.
if DIAG_SOURCE
    NSAMP = 8192;
end

FIELD_FILE = "fpga_state.txt";
FFT_FILE   = "fpga_spectrum.txt";

printf("------------------------------------------------\n");
printf("Courant number S        %.6g   (1/%d)\n", S, 1/S);
printf("physical time per step  %.6g\n", S);
printf("record length           %d steps = %.4g time units\n", NSAMP, NSAMP*S);
printf("source period           %g time units -> f0 = %.6g\n", WAVELEN, 1/WAVELEN);
printf("FFT resolution          %.6g\n", 1/(NSAMP*S));
printf("Nyquist                 %.6g\n", 1/(2*S));
printf("------------------------------------------------\n");

% ----------------------------------------------------------------
% open the port and start the solver
% ----------------------------------------------------------------

s = serial(SERIAL_PORT, BAUD_RATE);
set(s, "timeout", 10);          % deciseconds in some builds; see note below
fopen(s);
cleanup_obj = onCleanup(@() safe_close(s));

pause(0.3);
srl_flush(s);

fwrite(s, uint8("R"), "uint8");   % reset the core
pause(0.1);
if DIAG_SOURCE
    fwrite(s, uint8("D"), "uint8");
    printf("DIAGNOSTIC MODE: the state column is the SOURCE sample.\n");
else
    fwrite(s, uint8("F"), "uint8");
end
pause(0.1);
fwrite(s, uint8("G"), "uint8");   % run
printf("reset and run sent; collecting %d frames...\n", NSAMP);

% ----------------------------------------------------------------
% collect frames, resynchronising on the 0xA5 marker
% ----------------------------------------------------------------

sym   = zeros(1, NSAMP);
state = zeros(1, NSAMP);
stepb = zeros(1, NSAMP);
n     = 0;
nresync = 0;
ndup  = 0;
ndrop = 0;
prev  = -1;

% Once synchronised, read WHOLE frames rather than scanning for the
% sync byte every time.  The step byte takes every value in 0..255, so
% it equals 0xA5 once every 256 steps; a scan-for-sync reader locks
% onto those and reports a spurious duplicate each time (8192/256 = 32,
% exactly the count previously seen).  Resynchronise only when the sync
% byte is actually wrong.
synced = false;

while n < NSAMP

    if ~synced
        b = double(fread(s, 1, "uint8"));
        if isempty(b), continue; end
        if b != 165                 % 0xA5
            nresync = nresync + 1;
            continue;
        end
        synced = true;
    else
        b = double(fread(s, 1, "uint8"));
        if isempty(b), continue; end
        if b != 165
            synced  = false;        % lost framing; rescan
            nresync = nresync + 1;
            continue;
        end
    end

    payload = double(fread(s, 4, "uint8"));
    if numel(payload) < 4, synced = false; continue; end

    v = payload(1);
    if v >= 128, v = v - 256; end          % int8

    a = payload(2) * 256 + payload(3);
    if a >= 32768, a = a - 65536; end      % int16 big-endian

    sb = payload(4);

    % Continuity of the step byte. A genuinely duplicated or dropped
    % frame still decodes and still looks like a field; only the sample
    % rate is wrong, by an integer factor, which silently rescales the
    % whole frequency axis.
    if prev >= 0
        d = mod(sb - prev, 256);
        if d == 0
            ndup = ndup + 1;
            prev = sb;
            continue;
        elseif d > 1
            ndrop = ndrop + (d - 1);
        end
    end
    prev = sb;

    n = n + 1;
    sym(n)   = v;
    state(n) = a;
    stepb(n) = sb;

    if mod(n, 1000) == 0
        printf("  %d / %d\n", n, NSAMP);
    end
end

fwrite(s, uint8("S"), "uint8");   % stop
printf("done. %d resync bytes discarded.\n", nresync);
printf("duplicate frames discarded: %d\n", ndup);
printf("dropped samples detected:   %d\n", ndrop);
if ndup > 0 || ndrop > 0
    printf("\n  *** THE STREAM IS NOT ONE FRAME PER STEP.\n");
    printf("      Every frequency in the analysis below is wrong by\n");
    printf("      the corresponding factor.  Run  make sim-frame  and\n");
    printf("      fix the framing before trusting any of it.\n\n");
end

% ----------------------------------------------------------------
% save
% ----------------------------------------------------------------

fid = fopen(FIELD_FILE, "w");
fprintf(fid, "# step symbol state  (field = state/%d in units of A)\n", M);
for k = 1:NSAMP
    fprintf(fid, "%d %d %d\n", k-1, sym(k), state(k));
end
fclose(fid);
printf("state stream saved to %s\n", FIELD_FILE);

% ----------------------------------------------------------------
% diagnostic verdict
% ----------------------------------------------------------------

if DIAG_SOURCE
    SRC_AMP = SRC_K * 2^MC_LOG;
    printf("------------------------------------------------\n");
    printf("source ROM check\n");
    printf("  expected amplitude   %d\n", SRC_AMP);
    printf("  observed min / max   %d / %d\n", min(state), max(state));
    printf("  distinct values      %d\n", numel(unique(state)));
    if max(state) == min(state)
        printf("  VERDICT: FAIL -- the source is CONSTANT.  The ROM did\n");
        printf("           not initialise.  Check that build/sine_lut.hex\n");
        printf("           exists when yosys runs and that $readmemh uses a\n");
        printf("           string literal, not a parameter.\n");
    elseif abs(max(state) - SRC_AMP) > 0.05*SRC_AMP || min(state) >= 0
        printf("  VERDICT: FAIL -- wrong amplitude or no negative half.\n");
    else
        printf("  VERDICT: PASS -- the source ROM is correct.\n");
        printf("           Set DIAG_SOURCE = false and capture the field.\n");
    end
    printf("------------------------------------------------\n");
    figure("name","source sample"); plot(state(1:min(end,4096)),"-");
    xlabel("step"); ylabel("source sample (state units)");
    title("source ROM as the hardware sees it"); grid on;

    % Close the port EXPLICITLY before returning.  A bare `return` here
    % leaves onCleanup to fire after the script's local functions have
    % gone out of scope, so safe_close is undefined, the cleanup throws
    % and the port is never closed.  A serial port left open by a dead
    % script is not a cosmetic problem: the next opener reads nothing
    % for ever while every other layer looks healthy, which is exactly
    % the failure that made the board appear dead for several hours.
    try
        fclose(s);
    catch
    end
    clear cleanup_obj;
    printf("serial port closed.\n");
    return;
end

% ----------------------------------------------------------------
% spectra
%
% Two spectra, of two different quantities, and they must not be
% confused.  `state/M` is the state field a*A/M, the quantity that
% converges to the reference.  `sym` is the raw emitted symbol
% stream, with no reconstruction filter applied at all, whose
% spectrum shows the noise shaping directly: the quantisation error
% is pushed away from the source line towards the Nyquist frequency.
% ----------------------------------------------------------------

t   = (0:NSAMP-1) * S;
f   = (0:floor(NSAMP/2)) / (NSAMP * S);

Xs = fft(state/M - mean(state/M));
Ps = max(abs(Xs(1:numel(f))) / NSAMP, eps);  Ps(2:end-1) = 2*Ps(2:end-1);

Xy = fft(sym - mean(sym));
Py = max(abs(Xy(1:numel(f))) / NSAMP, eps);  Py(2:end-1) = 2*Py(2:end-1);

% Do NOT report the largest bin as "the measured frequency".  The grid
% is a lossless PEC box: once driven it rings its own eigenmodes for
% ever, and one of those routinely exceeds the drive line.  The honest
% quick-look number is the amplitude AT f0; run fdtd_spectrum.m for the
% proper analysis, which removes the turn-on transient and identifies
% the other peaks against the hard-wall eigenfrequencies.
f0 = 1 / WAVELEN;
[~, i0] = min(abs(f - f0));

printf("------------------------------------------------\n");
printf("source frequency f0            %.6g\n", f0);
printf("state-field amplitude at f0    %.6g\n", Ps(i0));
printf("record length                  %.1f source periods\n", NSAMP*S/WAVELEN);
printf("\nrun  octave host/fdtd_spectrum.m  for the full analysis\n");
printf("------------------------------------------------\n");

fid = fopen(FFT_FILE, "w");
fprintf(fid, "# frequency  amp_state_field  amp_raw_symbols\n");
for k = 1:numel(f)
    fprintf(fid, "%.12g %.12g %.12g\n", f(k), Ps(k), Py(k));
end
fclose(fid);
printf("spectra saved to %s\n", FFT_FILE);

% ----------------------------------------------------------------
% plots
% ----------------------------------------------------------------

figure("name", "state field at the probe");
plot(t, state/M, "-");
xlabel("physical time");
ylabel("a^{E} A / M   (units of A)");
title(sprintf("ternary state field, S = 1/%d", 1/S));
grid on;

figure("name", "raw symbol stream");
stairs(t, sym, "-");
ylim([-1.5 1.5]);
xlabel("physical time");
ylabel("emitted symbol s");
title("raw ternary symbols, no reconstruction filter");
grid on;

figure("name", "spectra");
semilogy(f, Ps, "-", "linewidth", 1); hold on;
semilogy(f, Py, "-");
plot([f0 f0], ylim(), "k--");
hold off;
xlabel("frequency (physical units)");
ylabel("amplitude");
legend("state field a A/M", "raw symbols s", "source f_0", ...
       "location", "northeast");
title("noise shaping: error is pushed towards Nyquist");
grid on;

% ----------------------------------------------------------------

