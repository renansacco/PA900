function [sys, x0, str, ts] = sf_veiculo(t, x, u, flag, X0, params)
% SF_VEICULO  Level-1 S-Function wrapper for the vehicle plant model.
%
%   [sys, x0, str, ts] = sf_veiculo(t, x, u, flag, X0, params)
%
%   This S-Function wraps dinamica_veiculo.m for use inside Simulink.
%
%   Extra parameters (passed via the S-Function block dialog):
%       X0     - initial state vector (7x1)
%       params - parameter struct (see dinamica_veiculo for fields)
%
%   States (7 continuous):  [x, y, psi, r, vy, omega_m, delta]
%   Inputs  (2):            [omega_m_ref, vx]
%   Outputs (8):            all states + sideslip [rad]
%
%   Date:   2026-06-09
%   Author: Renan / Claude

    persistent vx_last
    if isempty(vx_last), vx_last = 2.0; end

    switch flag

        case 0  % --- Initialization ---
            sizes = simsizes;
            sizes.NumContStates  = 7;
            sizes.NumDiscStates  = 0;
            sizes.NumOutputs     = 8;
            sizes.NumInputs      = 2;
            sizes.DirFeedthrough = 0;
            sizes.NumSampleTimes = 1;

            sys = simsizes(sizes);
            x0  = X0(:);
            str = [];
            ts  = [0 0];   % continuous sample time

        case 1  % --- Derivatives ---
            vx_last = u(2);
            sys = dinamica_veiculo(x, u, params);

        case 3  % --- Outputs ---
            beta = atan2(x(5), vx_last);
            sys = [x; beta];

        case {2, 4, 9}  % Unused flags
            sys = [];

        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end

end
