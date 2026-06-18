function [sys, x0, str, ts] = sf_veiculo_cinematico(t, x, u, flag, X0, params)
% SF_VEICULO_CINEMATICO  S-Function wrapper for kinematic vehicle model.
%
%   Drop-in replacement para sf_veiculo. Mesma interface:
%     States (7):   [x, y, psi, r, vy, omega_m, delta]
%     Inputs (2):   [omega_m_ref, vx]
%     Outputs (8):  [states(1:7), beta]
%
%   Usa dinamica_veiculo_cinematico.m (modelo cinematico puro,
%   identico ao vehicle_simulator.cpp do simulador embarcado).

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
            ts  = [0 0];

        case 1  % --- Derivatives ---
            sys = dinamica_veiculo_cinematico(x, u, params);

        case 3  % --- Outputs ---
            L = params.Lf + params.Lr;
            beta = atan(params.Lr * tan(x(7)) / L);
            sys = [x; beta];

        case {2, 4, 9}
            sys = [];

        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end

end
