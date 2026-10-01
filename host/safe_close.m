function safe_close(s)
% SAFE_CLOSE  Close a serial port, ignoring an already-closed port.
%   Kept as its own file so that onCleanup can still reach it after a
%   script returns.  A script-local function goes out of scope first,
%   the cleanup throws, and the port is left OPEN -- after which the
%   next opener reads nothing for ever while every other layer looks
%   healthy.  That failure cost several hours of this bring-up.
    try
        fclose(s);
    catch
    end
    disp("serial port closed.");
end
