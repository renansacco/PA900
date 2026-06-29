%% Varredura de velocidades x agressividade — mapa de ganhos v2
%
% Otimiza [K_psi, K_r] para cada (vx, agressividade).
% Salva Gains_Curva(n_vx, 2, 3) em Curva_Gains_Linear.mat.

clear; close all;

%% Paths
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));

%% Planta
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));

%% Varredura
vx_sweep = 0.5:0.5:4.0;
n_vx = length(vx_sweep);
n_aggr = 3;

Gains_Curva = zeros(n_vx, 2, n_aggr);
T_look      = zeros(1, n_aggr);
cfgs        = cell(1, n_aggr);
aggr_names  = {'suave', 'padrao', 'agressivo'};
colors_aggr = {'b', 'k', 'r'};

res_all = cell(1, n_aggr);

for ia = 1:n_aggr
    cfg = config_curva(ia);
    cfgs{ia} = cfg;
    T_look(ia) = cfg.T_look;

    fprintf('\n############################################################\n');
    fprintf('  AGRESSIVIDADE: %s (idx=%d)\n', cfg.aggr_name, ia);
    fprintf('  Q_psi=%.2f  Q_r=%.4f  R_ctrl=%.4f  T_look=%.1f  omega_sat=%.0f\n', ...
        cfg.Q_psi, cfg.Q_r, cfg.R_ctrl, cfg.T_look, cfg.omega_sat);
    fprintf('############################################################\n');

    Ks0 = [30, 40];
    opts = optimset('Display', 'final', 'MaxIter', 100, 'TolX', 1e-8);

    res = struct('vx', {}, 'K_psi', {}, 'K_r', {}, 'J', {}, ...
        'Pm', {}, 'tau_mf', {}, 'w_bw', {}, 'ts', {}, 'Mp', {}, ...
        'umax', {}, 'usat', {});

    for k = 1:n_vx
        vx = vx_sweep(k);

        fprintf('\n--- %s | vx = %.1f m/s ---\n', cfg.aggr_name, vx);

        cfg.plot = false;
        [Ks, Jopt] = fminsearch(@(ks) objetivo_curva(ks, p, vx, cfg), Ks0, opts);

        cfg.plot = false;
        [~, t, X, U] = objetivo_curva(Ks, p, vx, cfg);

        gamma0 = cfg.gamma0;
        e_psi = gamma0 - X(:, 3);
        info.ts       = settling_time(t, e_psi, 0.02 * abs(gamma0));
        info.Mp       = max(0, max(X(:,3)) - gamma0) / abs(gamma0);
        info.umax     = max(abs(U(:,1)));
        info.usat_pct = 100 * sum(abs(U(:,1)) >= cfg.omega_sat * 0.99) / length(t);

        mf = analise_malha_fechada(p, vx, Ks, cfg);

        res(k).vx     = vx;
        res(k).K_psi  = Ks(1);
        res(k).K_r    = Ks(2);
        res(k).J      = Jopt;
        res(k).Pm     = mf.Pm;
        res(k).tau_mf = mf.tau_mf;
        res(k).w_bw   = mf.w_bw;
        res(k).ts     = info.ts;
        res(k).Mp     = info.Mp;
        res(k).umax   = info.umax;
        res(k).usat   = info.usat_pct;

        Gains_Curva(k, :, ia) = [Ks(1), Ks(2)];
        Ks0 = Ks;

        fprintf('  K_psi=%.2f  K_r=%.2f  J=%.4f  Pm=%.1f  tau=%.3f  ts=%.2f\n', ...
            Ks(1), Ks(2), Jopt, mf.Pm, mf.tau_mf, info.ts);
    end

    res_all{ia} = res;

    %% Tabela resumo por agressividade
    fprintf('\n============================================================\n');
    fprintf('  RESUMO [%s] — gamma0=%.0f deg, omega_sat=%.0f\n', ...
        cfg.aggr_name, rad2deg(cfg.gamma0), cfg.omega_sat);
    fprintf('============================================================\n');
    fprintf('%5s %7s %7s %6s %8s %8s %6s %6s\n', ...
        'vx', 'K_psi', 'K_r', 'Pm', 'tau_mf', 'w_bw', 'ts', 'Mp%');
    for k = 1:n_vx
        fprintf('%5.1f %7.2f %7.2f %6.1f %8.3f %8.2f %6.2f %5.1f\n', ...
            res(k).vx, res(k).K_psi, res(k).K_r, ...
            res(k).Pm, res(k).tau_mf, res(k).w_bw, ...
            res(k).ts, res(k).Mp*100);
    end
end

%% Figura resumo comparativa
figure('Name', 'Comparacao agressividades');

subplot(2,2,1);
for ia = 1:n_aggr
    res = res_all{ia};
    plot([res.vx], [res.Pm], [colors_aggr{ia} 'o-'], 'LineWidth', 1.5, ...
        'MarkerFaceColor', colors_aggr{ia}, 'DisplayName', aggr_names{ia}); hold on;
end
ylabel('Pm [deg]'); xlabel('v_x [m/s]'); grid on;
legend('Location', 'best'); title('Margem de fase');

subplot(2,2,2);
for ia = 1:n_aggr
    res = res_all{ia};
    plot([res.vx], [res.tau_mf], [colors_aggr{ia} 'o-'], 'LineWidth', 1.5, ...
        'MarkerFaceColor', colors_aggr{ia}, 'DisplayName', aggr_names{ia}); hold on;
end
ylabel('\tau_{MF} [s]'); xlabel('v_x [m/s]'); grid on;
legend('Location', 'best'); title('Constante de tempo MF');

subplot(2,2,3);
for ia = 1:n_aggr
    res = res_all{ia};
    plot([res.vx], [res.K_psi], [colors_aggr{ia} 'o-'], 'LineWidth', 1.5, ...
        'DisplayName', sprintf('K_{\\psi} %s', aggr_names{ia})); hold on;
    plot([res.vx], [res.K_r], [colors_aggr{ia} 's--'], 'LineWidth', 1, ...
        'DisplayName', sprintf('K_r %s', aggr_names{ia}));
end
ylabel('Ganho'); xlabel('v_x [m/s]'); grid on;
legend('Location', 'best'); title('Ganhos otimizados');

subplot(2,2,4);
for ia = 1:n_aggr
    res = res_all{ia};
    cfg_ia = cfgs{ia};
    plot([res.vx], [res.umax], [colors_aggr{ia} 'o-'], 'LineWidth', 1.5, ...
        'MarkerFaceColor', colors_aggr{ia}, 'DisplayName', aggr_names{ia}); hold on;
    yline(cfg_ia.omega_sat, [colors_aggr{ia} ':'], 'HandleVisibility', 'off');
end
ylabel('\omega_m max [rad/s]'); xlabel('v_x [m/s]'); grid on;
legend('Location', 'best'); title('Pico de atuacao');

sgtitle(sprintf('Varredura v2 — \\gamma_0=%.0f° — suave / padrao / agressivo', ...
    rad2deg(cfgs{1}.gamma0)));

%% Salva mapa de ganhos
vx_table = vx_sweep;
gains_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'ERT', 'gains');
name = fullfile(gains_dir, 'Curva_Gains_Linear.mat');
save(name, 'Gains_Curva', 'vx_table', 'T_look', 'aggr_names', 'cfgs');
fprintf('\nGanhos salvos em: %s\n', name);
fprintf('  Gains_Curva: [%d x %d x %d]  (vx x ganhos x agressividade)\n', size(Gains_Curva));
fprintf('  T_look: [%.1f, %.1f, %.1f]\n', T_look);

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
end
