function metrics = calcular_metricas(out, t_descarte)
% CALCULAR_METRICAS  Computa metricas de desempenho a partir da saida do Simulink.
%
%   metrics = calcular_metricas(out, t_descarte)
%
%   Inputs:
%       out         — SimulationOutput do sim('modelClosedLoop')
%       t_descarte  — tempo inicial a descartar para metricas de regime [s]
%
%   Output:
%       metrics — struct com campos:
%           .e_lat_mean, .e_lat_max, .e_lat_rms
%           .e_lat_hist_edges, .e_lat_hist_counts
%           .ctrl_energy, .ctrl_smooth
%           .overshoot, .settling_time
%           .sat_omegam_pct, .sat_delta_pct
%           .max_omegam, .max_delta_deg

    OMEGAM_SAT = 10;    % rad/s (limite fisico pratico)
    DELTA_MAX  = 50;     % graus (limite fisico)

    %% Extrai sinais
    t           = out.e.time;
    e_lat       = out.e.signals.values;
    omegam_ref  = out.omegam_ref.signals.values;
    omegam      = out.omegam.signals.values;
    delta_deg   = out.delta_deg.signals.values;

    Ts = mean(diff(t));

    %% Indices de regime (apos descarte) e transiente
    idx_regime = t >= t_descarte;
    idx_trans  = t < 5;  % primeiros 5s para overshoot

    %% Erro lateral — regime
    e_abs = abs(e_lat(idx_regime));
    metrics.e_lat_mean = mean(e_abs);
    metrics.e_lat_max  = max(e_abs);
    metrics.e_lat_rms  = sqrt(mean(e_lat(idx_regime).^2));

    %% Histograma do erro lateral (toda a simulacao)
    edges = -2 : 0.05 : 2;  % bins de 5cm, range +-2m
    metrics.e_lat_hist_edges  = edges;
    metrics.e_lat_hist_counts = histcounts(e_lat, edges);

    %% Sinal de controle
    metrics.ctrl_energy = sqrt(mean(omegam_ref(idx_regime).^2));

    d_omegam_ref = diff(omegam_ref) / Ts;
    metrics.ctrl_smooth = sqrt(mean(d_omegam_ref(idx_regime(1:end-1)).^2));

    %% Transiente
    metrics.overshoot = max(abs(e_lat(idx_trans)));

    e_abs_full = abs(e_lat);
    settled = e_abs_full < 0.10;
    % Tempo de acomodacao: primeiro instante a partir do qual permanece < 0.10m
    idx_settle = find(~settled, 1, 'last');
    if isempty(idx_settle)
        metrics.settling_time = 0;
    elseif idx_settle >= numel(t)
        metrics.settling_time = t(end);  % nunca acomodou
    else
        metrics.settling_time = t(idx_settle + 1);
    end

    %% Saturacao
    metrics.sat_omegam_pct = 100 * sum(abs(omegam) >= OMEGAM_SAT) / numel(omegam);
    metrics.sat_delta_pct  = 100 * sum(abs(delta_deg) >= DELTA_MAX) / numel(delta_deg);

    %% Maximos absolutos
    metrics.max_omegam   = max(abs(omegam));
    metrics.max_delta_deg = max(abs(delta_deg));

end
