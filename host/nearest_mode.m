function [fm, mbest, nbest] = nearest_mode(fx, nx)
% NEAREST_MODE  Closest hard-wall cavity eigenfrequency to fx.
%   Uses k_x = m*pi/((Nx-1)*dx), the HARD-WALL convention, so
%   f_mn = sqrt(m^2 + n^2) / (2*(Nx-1)).  Not the periodic convention.
    best = Inf;
    mbest = 1; nbest = 1;
    fm = hypot(1,1) / (2*(nx-1));
    for m = 1:8
      for n = 1:8
        fmn = hypot(m, n) / (2*(nx-1));
        if abs(fmn - fx) < best
            best  = abs(fmn - fx);
            mbest = m; nbest = n; fm = fmn;
        end
      end
    end
end
