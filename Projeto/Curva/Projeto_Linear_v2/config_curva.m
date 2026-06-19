function [cfg, gamma0_design, T_look] = config_curva()
% Configuracao compartilhada do projeto de controlador de curva.
% Usado por Projeto_Curva_Linear.m e varredura_velocidade.m.

gamma0_design = deg2rad(10);      % erro de heading de projeto [rad]
T_look        = 2.0;             % tempo de lookahead [s]

cfg.tsim      = 15;             % tempo de simulacao [s]
cfg.omega_sat = 6;              % saturacao omega_m [rad/s]
cfg.X0        = zeros(7, 1);    % equilibrio
cfg.Pm_min    = 35;             % margem de fase minima [deg]
cfg.tau_ratio_max = 0.30;       % tau_MF / tau_la maximo
cfg.R_ctrl    = 1.0 / cfg.omega_sat^2;

r_max     = deg2rad(15);        % yaw rate maximo toleravel [rad/s]
cfg.Q_r   = 1 / r_max^2;

end
