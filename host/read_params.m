function p = read_params(fname)
% READ_PARAMS  Read build/params.vh, which the C golden model writes.
%
%   The physical parameters must come from ONE place.  Duplicating
%   MC_LOG in this script and in the build silently analyses a capture
%   with the wrong Courant number: every frequency, the transient
%   estimate and the whole spectrum scale by the ratio of the two, and
%   nothing in the output looks wrong.
%
%   Returns a struct with NX, NY, M_LOG, MC_LOG, W, PHASE_INC, NSTEP,
%   PX, PY, plus the derived S and WAVELEN.
    if nargin < 1, fname = "build/params.vh"; end
    fid = fopen(fname, "r");
    if fid < 0
        error(["cannot open %s -- run 'make hw-params' (or any build) " ...
               "so the golden model regenerates it"], fname);
    end
    p = struct();
    while true
        l = fgetl(fid);
        if ~ischar(l), break; end
        t = regexp(strtrim(l), '^`define\s+FDTD_(\w+)\s+(-?\d+)', "tokens");
        if ~isempty(t)
            p.(t{1}{1}) = str2double(t{1}{2});
        end
    end
    fclose(fid);

    p.S = 2^(p.MC_LOG - p.M_LOG);
    p.M = 2^p.M_LOG;

    % Physical source period: the phase accumulator is PHASE_BITS = 16
    % wide, so one source period is 2^16/PHASE_INC steps, and each step
    % is S units of physical time.
    p.WAVELEN = (65536 / p.PHASE_INC) * p.S;
end
