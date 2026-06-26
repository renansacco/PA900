function cfg = config_curva()
% Configuracao compartilhada do projeto de controlador de curva.
% Usado por Projeto_Curva_Linear.m e varredura_velocidade.m.

cfg.gamma0    = deg2rad(5);    % step de heading de projeto [rad]
cfg.tsim      = 15;             % tempo de simulacao [s]
cfg.omega_sat = 8;              % saturacao omega_m [rad/s]
cfg.X0        = zeros(7, 1);    % equilibrio
cfg.Pm_min    = 35;             % margem de fase minima [deg]

cfg.T_look = 2; % Tlook em segundos - define a malha externa e tau_mf_targer

cfg.Ms_max    = 0; %1.6;           % pico de sensibilidade max |S|  (1.4 p/ ~PM45)
cfg.tau_mf_tol = 0.20;
cfg.tau_mf_target  = 0; %0.6;          % tau_MF maximo [s]


% Pesos da função de custo
cfg.R_ctrl    = 1.0 / cfg.omega_sat^2;
cfg.Q_psi     = 1.0 / deg2rad(5)^2;       
cfg.Q_r   = 1 / deg2rad(30)^2; % yaw rate maximo toleravel [rad/s]

end
