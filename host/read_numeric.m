function a = read_numeric(fname)
% READ_NUMERIC  Tolerant whitespace-separated numeric reader.
%   Skips '#' and '%' comment lines.  textscan is core Octave; the
%   fgetl loop is the fallback for older builds.
    fid = fopen(fname, "r");
    if fid < 0
        error("cannot open %s", fname);
    end
    a = [];
    try
        c = textscan(fid, "%f %f %f", "CommentStyle", "#");
        if ~isempty(c) && ~isempty(c{1})
            a = [c{1}, c{2}, c{3}];
        end
    catch
        a = [];
    end
    if isempty(a)
        frewind(fid);
        rows = {};
        while true
            l = fgetl(fid);
            if ~ischar(l), break; end
            l = strtrim(l);
            if isempty(l) || l(1) == '#' || l(1) == '%', continue; end
            v = sscanf(l, "%f")';
            if ~isempty(v), rows{end+1} = v; end
        end
        a = cell2mat(rows(:));
    end
    fclose(fid);
end
