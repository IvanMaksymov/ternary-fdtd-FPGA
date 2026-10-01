% ====================================================================
% fdtd_spectrum.m -- OFFLINE analysis of a captured probe stream.
%
% Reads fpga_state.txt, which fdtd_capture.m wrote, and does the
% spectral analysis with no serial port involved.  Re-run it as often
% as you like with different windows, transient cuts and record
% lengths; the board is not needed.
%
% THREE THINGS THIS SCRIPT DOES THAT A BARE fft() DOES NOT.
%
% 1.  It discards the turn-on transient.  The wave needs
%     |probe - source| / S steps to reach the probe, and the record
%     before that is identically zero.  A step of that size puts more
%     energy in the lowest bin than the source line carries, so the
%     largest bin is bin 1 and the "measured frequency" comes out at
%     the FFT resolution rather than at f0.
%
% 2.  It checks that the record is long enough.  Resolution is
%     1/(N*S) and the source sits at 1/WAVELEN, so a record of P
%     source periods puts the drive line in bin P.  Fewer than about
%     20 periods and the line cannot be separated from the cavity
%     modes next to it.
%
% 3.  It does not report the largest bin as "the frequency".  The grid
%     is a lossless PEC box, so once driven it rings its own
%     eigenmodes indefinitely, and one of those can easily exceed the
%     drive line.  The right statements are the amplitude AT f0 and
%     the identification of the other peaks against the hard-wall
%     eigenfrequencies
%
%         f_mn = sqrt(m^2 + n^2) / (2 * (Nx - 1))
%
%     which uses k_x = m*pi/((Nx-1)*dx), the hard-wall convention, NOT
%     the periodic one.
% ====================================================================

clear; close all;

% Octave requires each function in its own file unless the script is
% forced into script mode, so read_numeric and nearest_mode live in
% host/.  Add that directory to the path when run from the project root.
if exist("host", "dir"), addpath("host"); end

% ----------------------------------------------------------------
% configuration -- must match the Makefile's HW_* values
% ----------------------------------------------------------------

% Which capture to analyse.
%   make spectrum             -> fpga_state.txt from a plain `make capture`
%   make spectrum MC=5        -> fpga_state_mc5.txt from `make sweep`
% The sweep RENAMES fpga_state.txt to fpga_state_mc<N>.txt, so after a
% sweep there is no fpga_state.txt to open.  Each sweep capture carries
% its own parameter file, and that is the one read here: analysing a
% capture with another run's S rescales every frequency in the result.
mcsel = getenv("FDTD_MC");
if ~isempty(mcsel)
    INFILE  = sprintf("fpga_state_mc%s.txt",  mcsel);
    PARFILE = sprintf("fpga_params_mc%s.vh",  mcsel);
elseif exist("fpga_state.txt", "file")
    INFILE  = "fpga_state.txt";
    PARFILE = "build/params.vh";
else
    d = dir("fpga_state_mc*.txt");
    if isempty(d)
        error(["no capture found.  Run `make capture`, or after a sweep " ...
               "use `make spectrum MC=5` to pick one of fpga_state_mc*.txt"]);
    end
    INFILE  = d(1).name;
    mc      = strrep(strrep(INFILE, "fpga_state_mc", ""), ".txt", "");
    PARFILE = sprintf("fpga_params_mc%s.vh", mc);
    printf("no fpga_state.txt; using %s\n", INFILE);
end
if ~exist(PARFILE, "file")
    error("%s exists but %s does not -- cannot know its Courant number",
          INFILE, PARFILE);
end
printf("capture %s   parameters %s\n", INFILE, PARFILE);

% Parameters come from build/params.vh, written by the golden model.
PAR     = read_params(PARFILE);
NX      = PAR.NX;
M_LOG   = PAR.M_LOG;
MC_LOG  = PAR.MC_LOG;
WAVELEN = PAR.WAVELEN;

NY = NX;                % the core is square; change both if it is not

SX = NX/2;  SY = NY/2;  % source cell
PX = NX/4;  PY = NY/4;  % probe cell

SKIP_EXTRA_PERIODS = 2; % settle time discarded after the wave arrives
USE_WINDOW         = true;
FMAX_PLOT          = 0.15;             % plot range, physical units

S  = 2^(MC_LOG - M_LOG);
M  = 2^M_LOG;
f0 = 1 / WAVELEN;

% ----------------------------------------------------------------
% load
% ----------------------------------------------------------------

raw = read_numeric(INFILE);
if isempty(raw)
    error("no data in %s", INFILE);
end

step  = raw(:,1)';
sym   = raw(:,2)';
state = raw(:,3)' / M;          % field units of A

Ntot = numel(state);

% ----------------------------------------------------------------
% transient
% ----------------------------------------------------------------

travel     = hypot(SX-PX, SY-PY);              % cells
arrive_est = ceil(travel / S);                 % steps

nz = find(state ~= 0, 1);
if isempty(nz), nz = arrive_est; end

MINSAMP   = round(4 * WAVELEN / S);        % refuse to analyse under 4 periods
skip_want = nz + SKIP_EXTRA_PERIODS * round(WAVELEN / S);

short_record = false;
if skip_want + MINSAMP <= Ntot
    skip = skip_want;
elseif nz + MINSAMP <= Ntot
    skip = nz;                              % no settle time to spare
    printf("  note: record too short to discard settle time as well\n");
else
    skip = nz;
    short_record = true;
end

N = Ntot - skip;
if N < 16
    error(["the record is %d samples and the wave only reaches the probe " ...
           "at step %d, so there is nothing to analyse.  Increase NSAMP " ...
           "in fdtd_capture.m to at least %d."], Ntot, nz, ...
          nz + round(40*WAVELEN/S));
end
x  = state(skip+1:end);
sy = sym(skip+1:end);

periods = N * S / WAVELEN;

printf("------------------------------------------------\n");
printf("file                    %s\n", INFILE);
printf("samples in file         %d\n", Ntot);
printf("Courant number S        %.6g   (1/%d)\n", S, round(1/S));
printf("wave arrives at step    %d  (predicted %d from %.1f cells)\n", ...
       nz, arrive_est, travel);
printf("transient discarded     %d steps\n", skip);
printf("record analysed         %d steps = %.4g time units = %.1f periods\n", ...
       N, N*S, periods);
printf("FFT resolution          %.6g   (source is bin %d)\n", ...
       1/(N*S), round(f0*N*S));
printf("Nyquist                 %.6g\n", 1/(2*S));
if short_record
    printf("\n  *** RECORD TOO SHORT.  The wave reaches the probe at step\n");
    printf("      %d and only %d samples follow.  Everything below is\n", nz, N);
    printf("      dominated by the turn-on step, and the largest bin will\n");
    printf("      be bin 1 rather than the source line.\n");
end
if periods < 20
    printf("\n  *** only %.1f source periods: the drive line cannot be\n", periods);
    printf("      separated from the cavity modes.  Capture at least\n");
    printf("      %d samples (NSAMP in fdtd_capture.m) for 40 periods.\n", ...
           skip + round(40*WAVELEN/S));
end
printf("------------------------------------------------\n");

% ----------------------------------------------------------------
% spectra
%
% Two different quantities, kept apart deliberately:
%   state/M  the state field a*A/M, which converges to the reference
%   sym      the raw emitted symbols, no reconstruction filter at all
% ----------------------------------------------------------------

if USE_WINDOW
    w = 0.5 * (1 - cos(2*pi*(0:N-1)/(N-1)));   % Hann, written out
else
    w = ones(1, N);
end
cg = mean(w);                                   % coherent gain

nf = floor(N/2) + 1;
f  = (0:nf-1) / (N * S);

Ps = fft((x  - mean(x )) .* w);  Ps = abs(Ps(1:nf)) / (N*cg/2);
Py = fft((sy - mean(sy)) .* w);  Py = abs(Py(1:nf)) / (N*cg/2);

% ----------------------------------------------------------------
% amplitude at the drive line, and peak identification
% ----------------------------------------------------------------

[~, i0] = min(abs(f - f0));
printf("amplitude of the state field at f0 = %.5g : %.5g\n", f0, Ps(i0));

printf("\npeaks below %.3g, against the hard-wall eigenfrequencies\n", FMAX_PLOT);
printf("  f_peak     amplitude   mode    f_mn       error\n");

sel  = find(f > 0 & f < FMAX_PLOT);
thr  = 0.02 * max(Ps(sel));
npk  = 0;
for k = sel(2:end-1)
    if Ps(k) > Ps(k-1) && Ps(k) > Ps(k+1) && Ps(k) > thr
        [fm, mm, nn] = nearest_mode(f(k), NX);
        if abs(f(k) - f0) < 1.5/(N*S)
            printf("  %.5f   %.5f     --      (source f0)\n", f(k), Ps(k));
        else
            printf("  %.5f   %.5f     (%d,%d)   %.5f   %+.2f %%\n", ...
                   f(k), Ps(k), mm, nn, fm, 100*(f(k)-fm)/fm);
        end
        npk = npk + 1;
    end
end
if npk == 0
    printf("  (none above threshold)\n");
end
printf("------------------------------------------------\n");

% ----------------------------------------------------------------
% plots.  Zeros are floored before the log axis: semilogy silently
% drops non-positive points and warns for every one of them.
% ----------------------------------------------------------------

flo = max(Ps, eps);
fly = max(Py, eps);

t = (0:N-1) * S;

figs = {};

h1 = figure("name", "state field at the probe");
plot(t, x, "-");
xlabel("physical time");
ylabel("a^{E} A / M   (units of A)");
title(sprintf("ternary state field, S = 1/%d, transient removed", round(1/S)));
grid on;
figs{end+1} = {h1, "waveform"};

h2 = figure("name", "spectra");
semilogy(f, flo, "-", "linewidth", 1.2); hold on;
semilogy(f, fly, "-");

yl = ylim();
plot([f0 f0], yl, "k--", "linewidth", 1.2);
for m = 1:5
  for n = 1:5
    fm = hypot(m,n) / (2*(NX-1));
    if fm < FMAX_PLOT
        plot([fm fm], [yl(1) yl(1)*30], "-", "color", [0.6 0.6 0.6]);
    end
  end
end
hold off;

xlim([0 FMAX_PLOT]);
ylim(yl);
xlabel("frequency (physical units)");
ylabel("amplitude");
legend("state field  a A / M", "raw symbols  s", "source f_0", ...
       "location", "southwest");
title("cavity modes and the drive line; error shaped towards Nyquist");
grid on;
figs{end+1} = {h2, "spectrum"};

h3 = figure("name", "full band");
semilogy(f, flo, "-"); hold on;
semilogy(f, fly, "-"); hold off;
xlabel("frequency (physical units)");
ylabel("amplitude");
legend("state field", "raw symbols", "location", "southwest");
title("full band to Nyquist: the shaped quantisation error rises with f");
grid on;
figs{end+1} = {h3, "fullband"};


% ----------------------------------------------------------------
% Save the figures.  Octave exits when a script ends and takes its
% windows with it, so a figure that is only displayed is lost the
% moment the script returns.  Saving is also what makes the result
% reproducible: the PNGs go in the repository, the screen does not.
% Use  octave --persist  (which `make spectrum` does) to keep the
% windows open for inspection as well.
% ----------------------------------------------------------------

tag = strrep(strrep(INFILE, "fpga_state", ""), ".txt", "");
if isempty(tag), tag = ""; end

for k = 1:numel(figs)
    if ishandle(figs{k}{1})
        fn = sprintf("fig_%s%s.png", figs{k}{2}, tag);
        try
            print(figs{k}{1}, fn, "-dpng", "-r150");
            printf("saved %s\n", fn);
        catch err
            printf("could not save %s (%s)\n", fn, err.message);
        end
    end
end
