function cfg = config_curva(aggr_idx)
% Configuracao do projeto de controlador de curva v2.
%
%   cfg = config_curva()        — retorna config padrao (aggr_idx=2)
%   cfg = config_curva(aggr_idx) — 1=suave, 2=padrao, 3=agressivo

if nargin < 1, aggr_idx = 2; end

cfg.aggr_names = {'suave', 'padrao', 'agressivo'};
cfg.aggr_idx   = aggr_idx;
cfg.aggr_name  = cfg.aggr_names{aggr_idx};

%% Parametros comuns
cfg.gamma0    = deg2rad(5);
cfg.tsim      = 15;
cfg.X0        = zeros(7, 1);
%cfg.Pm_min    = 35;
cfg.Ms_max    = 0;
cfg.tau_mf_tol = 0.20;

switch aggr_idx
    case 1  % suave
        cfg.Q_psi         = 1.0 / deg2rad(8)^2;
        cfg.Q_r           = 1.0 / deg2rad(30)^2;
        cfg.omega_sat     = 7;
        cfg.R_ctrl        = 1.0 / cfg.omega_sat^2;
        cfg.T_look        = 2;
        cfg.tau_mf_target = 0;

    case 2  % padrao
        cfg.Q_psi         = 1.0 / deg2rad(8)^2;
        cfg.Q_r           = 1.0 / deg2rad(30)^2;
        cfg.omega_sat     = 9;
        cfg.R_ctrl        = 1.0 / cfg.omega_sat^2;
        cfg.T_look        = 2;
        cfg.tau_mf_target = 0;

    case 3  % agressivo
        cfg.Q_psi         = 1.0 / deg2rad(8)^2;
        cfg.Q_r           = 1.0 / deg2rad(30)^2;
        cfg.omega_sat     = 11;
        cfg.R_ctrl        = 1.0 / cfg.omega_sat^2;
        cfg.T_look        = 2;
        cfg.tau_mf_target = 0;
end

end
