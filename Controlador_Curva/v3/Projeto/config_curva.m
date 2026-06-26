function cfg = config_curva()
% Configuracao do projeto de controlador de curva v3.
% Diferenca para v2: realimentacao de delta (K_delta).

cfg.gamma0    = deg2rad(5);
cfg.tsim      = 15;
cfg.omega_sat = 15;
cfg.X0        = zeros(7, 1);
cfg.Pm_min    = 35;

cfg.T_look    = 2;

cfg.Ms_max        = 0;
cfg.tau_mf_tol    = 0.20;
cfg.tau_mf_target = 0;

% Pesos da funcao de custo
cfg.R_ctrl = 1.0 / cfg.omega_sat^2;
cfg.Q_psi  = 1.0 / deg2rad(4)^2;
cfg.Q_r    = 1.0 / deg2rad(30)^2;

end
