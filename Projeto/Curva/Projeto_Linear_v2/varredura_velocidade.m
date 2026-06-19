%% Varredura de velocidades — mapa de ganhos do controlador de curva
%
% Otimiza [K_psi, K_r] para cada vx com Delta = vx * T_look.
% Config carregada de config_curva.m.

clear; close all;

%% Paths
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));

%% Planta e config
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));
[cfg, gamma0_design, T_look] = config_curva();

%% Varredura
vx_sweep = 0.5:0.5:4.0;
n_vx = length(vx_sweep);

res = struct('vx', {}, 'Delta', {}, 'K_psi', {}, 'K_r', {}, 'J', {}, ...
    'Pm', {}, 'tau_mf_ratio', {}, 'omega_ratio', {}, 'ts', {}, 'Mp', {}, ...
    'umax', {}, 'usat', {});

Ks0 = [30, 40];
opts = optimset('Display', 'iter', 'MaxIter', 50, 'TolX', 1e-8);

for k = 1:n_vx
    vx = vx_sweep(k);
    cfg.Delta = vx * T_look;
    cfg.e0    = abs(gamma0_design) * cfg.Delta;
    gamma0    = -cfg.e0 / cfg.Delta;
    cfg.Q_psi = 1.0 / (gamma0)^2;

    fprintf('\n========================================\n');
    fprintf('  vx = %.1f m/s, Delta = %.1f m (T_look=%.1fs)\n', vx, cfg.Delta, T_look);
    fprintf('========================================\n');

    % Otimizacao
    cfg.plot = false;
    [Ks, Jopt] = fminsearch(@(ks) objetivo_curva(ks, p, vx, cfg), Ks0, opts);

    fprintf('\n=== Resultado vx=%.1f ===\n', vx);
    fprintf('  K_psi = %.4f\n', Ks(1));
    fprintf('  K_r   = %.4f\n', Ks(2));
    fprintf('  J     = %.6f\n', Jopt);

    % Simula resultado final com plot
    cfg.plot = true;
    [~, t, X, U] = objetivo_curva(Ks, p, vx, cfg);
    set(gcf, 'Name', sprintf('Simulacao vx=%.0f', vx));

    % Metricas
    tau_la = cfg.Delta / vx;
    psi_ref_final = gamma0 * exp(-t / tau_la);
    e_psi = psi_ref_final - X(:, 3);
    info.ts       = settling_time(t, e_psi, 0.02 * abs(gamma0));
    info.Mp       = max(0, max(e_psi)) / abs(e_psi(1));
    info.umax     = max(abs(U(:,1)));
    info.usat_pct = 100 * sum(abs(U(:,1)) >= cfg.omega_sat * 0.99) / length(t);

    fprintf('\n=== Desempenho vx=%.1f ===\n', vx);
    fprintf('  Settling time (2%%): %.2f s\n', info.ts);
    fprintf('  Overshoot:          %.1f%%\n', info.Mp * 100);
    fprintf('  omega_m max:        %.1f rad/s\n', info.umax);
    fprintf('  Saturacao:          %.1f%% do tempo\n', info.usat_pct);

    % Analise MF
    mf = analise_malha_fechada(p, vx, Ks, cfg);

    % Salva resultados
    res(k).vx = vx;
    res(k).Delta = cfg.Delta;
    res(k).K_psi = Ks(1);
    res(k).K_r = Ks(2);
    res(k).J = Jopt;
    res(k).Pm = mf.Pm;
    res(k).tau_mf_ratio = mf.tau_mf / tau_la;
    res(k).omega_ratio = (vx / cfg.Delta) / mf.w_bw;
    res(k).ts = info.ts;
    res(k).Mp = info.Mp;
    res(k).umax = info.umax;
    res(k).usat = info.usat_pct;

    Ks0 = Ks;
end

%% Tabela resumo
fprintf('\n\n============================================================\n');
fprintf('  RESUMO — T_look=%.1fs, gamma0_design=%.0f deg\n', T_look, rad2deg(gamma0_design));
fprintf('============================================================\n');
fprintf('%5s %6s %7s %7s %6s %10s %10s %6s %6s\n', ...
    'vx', 'Delta', 'K_psi', 'K_r', 'Pm', 'tau/tau_la', 'wla/wbw', 'ts', 'Mp%');
for k = 1:n_vx
    fprintf('%5.1f %6.1f %7.2f %7.2f %6.1f %10.2f %10.2f %6.2f %5.1f\n', ...
        res(k).vx, res(k).Delta, res(k).K_psi, res(k).K_r, ...
        res(k).Pm, res(k).tau_mf_ratio, res(k).omega_ratio, ...
        res(k).ts, res(k).Mp*100);
end

%% Figura resumo
figure('Name', 'Resumo vs vx');

subplot(2,2,1);
plot([res.vx], [res.Pm], 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
yline(cfg.Pm_min, 'r--', sprintf('Pm_{min}=%d°', cfg.Pm_min));
ylabel('Pm [deg]'); xlabel('v_x [m/s]'); grid on;
title('Margem de fase');

subplot(2,2,2);
plot([res.vx], [res.tau_mf_ratio], 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
yline(cfg.tau_ratio_max, 'r--', sprintf('max=%.2f', cfg.tau_ratio_max));
ylabel('\tau_{MF} / \tau_{la}'); xlabel('v_x [m/s]'); grid on;
title('Razao de tempo (bandwidth)');

subplot(2,2,3);
plot([res.vx], [res.K_psi], 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b'); hold on;
plot([res.vx], [res.K_r], 'rs-', 'LineWidth', 1.5, 'MarkerFaceColor', 'r');
ylabel('Ganho'); xlabel('v_x [m/s]'); grid on;
legend('K_\psi', 'K_r', 'Location', 'best');
title('Ganhos otimizados');

subplot(2,2,4);
plot([res.vx], [res.umax], 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
yline(cfg.omega_sat, 'r--', '\omega_{sat}');
ylabel('\omega_m max [rad/s]'); xlabel('v_x [m/s]'); grid on;
title('Pico de atuacao');

sgtitle(sprintf('Varredura vx — T_{look}=%.1fs, \\gamma_0=%.0f°', T_look, rad2deg(gamma0_design)));

%% Salva mapa de ganhos — formato compativel com Param_Controller
% Gains_Curva(iv, ig): iv=indice de velocidade, ig=1:K_psi, 2:K_r
vx_table = vx_sweep;
Gains_Curva = zeros(n_vx, 2);
for k = 1:n_vx
    Gains_Curva(k, :) = [res(k).K_psi, res(k).K_r];
end

Desc = sprintf('Ganhos lineares curva. T_look=%.1fs, gamma0=%.0f deg, omega_sat=%.0f rad/s, Pm_min=%.0f deg, tau_ratio_max=%.2f', ...
    T_look, rad2deg(gamma0_design), cfg.omega_sat, cfg.Pm_min, cfg.tau_ratio_max);

gains_dir = fullfile(proj_root, 'Controller', 'gains');
name = fullfile(gains_dir, 'Curva_Gains_Linear.mat');
save(name, 'Gains_Curva', 'vx_table', 'T_look', 'Desc');
fprintf('\nGanhos salvos em: %s\n', name);

%% -----------------------------------------------------------------------
function ts = settling_time(t, e, tol)
    idx = find(abs(e) > tol, 1, 'last');
    if isempty(idx)
        ts = 0;
    else
        ts = t(idx);
    end
end

function mf = analise_malha_fechada(p, vx, Ks, cfg)
    K_psi = Ks(1);
    K_r   = Ks(2);
    tau_la = cfg.Delta / vx;

    Xe = zeros(7, 1);
    Ue = [0; vx];
    [A, B] = linearizar_veiculo(Xe, Ue, p);

    idx = 3:7;
    Ar = A(idx, idx);
    Br = B(idx, 1);

    K_fb = [K_psi, K_r, 0, 0, 0];
    A_mf = Ar - Br * K_fb;
    B_mf = Br * K_psi;
    C_mf = [1 0 0 0 0];

    sys_T = ss(A_mf, B_mf, C_mf, 0);
    sys_L = ss(Ar, Br, K_fb, 0);

    w_vec = logspace(-2, 2, 2000);
    [mag_T, ~] = bode(sys_T, w_vec);
    mag_T_dB = 20*log10(squeeze(mag_T));
    idx_bw = find(mag_T_dB < mag_T_dB(1) - 3, 1, 'first');
    if ~isempty(idx_bw)
        w_bw = w_vec(idx_bw);
    else
        w_bw = w_vec(end);
    end

    [~, Pm] = margin(sys_L);

    mf.Pm = Pm;
    mf.w_bw = w_bw;
    mf.tau_mf = 1 / w_bw;

    fprintf('\n=== Malha Fechada (vx=%.1f) ===\n', vx);
    fprintf('  Bandwidth: %.2f rad/s, tau_MF: %.2f s\n', w_bw, 1/w_bw);
    fprintf('  tau_MF/tau_la: %.2f, Pm: %.1f deg\n', (1/w_bw)/tau_la, Pm);
end
