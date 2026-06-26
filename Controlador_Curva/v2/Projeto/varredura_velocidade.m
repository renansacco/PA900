%% Varredura de velocidades — mapa de ganhos do controlador de curva
%
% Otimiza [K_psi, K_r] para cada vx.
% Referencia: step de heading gamma0.
% Config carregada de config_curva.m.

clear; close all;

%% Paths
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));

%% Planta e config
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));
cfg = config_curva();

%% Varredura
vx_sweep = 0.5:0.5:4.0;
n_vx = length(vx_sweep);

res = struct('vx', {}, 'K_psi', {}, 'K_r', {}, 'J', {}, ...
    'Pm', {}, 'tau_mf', {}, 'w_bw', {}, 'ts', {}, 'Mp', {}, ...
    'umax', {}, 'usat', {});

Ks0 = [30, 40];
opts = optimset('Display', 'iter', 'MaxIter', 50, 'TolX', 1e-8);

for k = 1:n_vx
    vx = vx_sweep(k);

    fprintf('\n========================================\n');
    fprintf('  vx = %.1f m/s\n', vx);
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
    gamma0 = cfg.gamma0;
    e_psi = gamma0 - X(:, 3);
    info.ts       = settling_time(t, e_psi, 0.02 * abs(gamma0));
    info.Mp       = max(0, max(X(:,3)) - gamma0) / abs(gamma0);
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
    res(k).K_psi = Ks(1);
    res(k).K_r = Ks(2);
    res(k).J = Jopt;
    res(k).Pm = mf.Pm;
    res(k).tau_mf = mf.tau_mf;
    res(k).w_bw = mf.w_bw;
    res(k).ts = info.ts;
    res(k).Mp = info.Mp;
    res(k).umax = info.umax;
    res(k).usat = info.usat_pct;

    Ks0 = Ks;
end

%% Tabela resumo
fprintf('\n\n============================================================\n');
fprintf('  RESUMO — gamma0=%.0f deg, tau_mf_max=%.2f s\n', ...
    rad2deg(cfg.gamma0), cfg.tau_mf_target);
fprintf('============================================================\n');
fprintf('%5s %7s %7s %6s %8s %8s %6s %6s\n', ...
    'vx', 'K_psi', 'K_r', 'Pm', 'tau_mf', 'w_bw', 'ts', 'Mp%');
for k = 1:n_vx
    fprintf('%5.1f %7.2f %7.2f %6.1f %8.3f %8.2f %6.2f %5.1f\n', ...
        res(k).vx, res(k).K_psi, res(k).K_r, ...
        res(k).Pm, res(k).tau_mf, res(k).w_bw, ...
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
plot([res.vx], [res.tau_mf], 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
yline(cfg.tau_mf_target, 'r--', sprintf('\\tau_{max}=%.2fs', cfg.tau_mf_target));
ylabel('\tau_{MF} [s]'); xlabel('v_x [m/s]'); grid on;
title('Constante de tempo MF');

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

sgtitle(sprintf('Varredura vx — \\gamma_0=%.0f°, \\tau_{mf,max}=%.2fs', ...
    rad2deg(cfg.gamma0), cfg.tau_mf_target));

%% Salva mapa de ganhos
vx_table = vx_sweep;
Gains_Curva = zeros(n_vx, 2);
for k = 1:n_vx
    Gains_Curva(k, :) = [res(k).K_psi, res(k).K_r];
end

Desc = sprintf('Ganhos lineares curva. gamma0=%.0f deg, omega_sat=%.0f rad/s, Pm_min=%.0f deg, tau_mf_max=%.2fs', ...
    rad2deg(cfg.gamma0), cfg.omega_sat, cfg.Pm_min, cfg.tau_mf_target);

gains_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'ERT', 'gains');
name = fullfile(gains_dir, 'Curva_Gains_Linear.mat');
save(name, 'Gains_Curva', 'vx_table', 'Desc', 'cfg');
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
    fprintf('  Bandwidth: %.2f rad/s, tau_MF: %.3f s\n', w_bw, 1/w_bw);
    fprintf('  Pm: %.1f deg\n', Pm);
end
