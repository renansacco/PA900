%% Varredura de tracking alpha(t) por velocidade
%
% Carrega ganhos do .mat de varredura_velocidade e simula o tracking
% de alpha(t) com feedforward para cada vx. Plota metricas de erro.

clear; close all;

%% Paths
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));

%% Carrega ganhos e planta
gains_file = fullfile(fileparts(mfilename('fullpath')), '..', 'ERT', 'gains', 'Curva_Gains_Linear.mat');
g = load(gains_file);
vx_table    = g.vx_table;
Gains_Curva = g.Gains_Curva;
n_vx = length(vx_table);

p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));

fprintf('Ganhos carregados: %s\n', gains_file);
fprintf('  vx = '); fprintf('%.1f ', vx_table); fprintf('\n');

%% Trajetoria: alpha(s) da taipa real (computado uma vez)
tmp = load('taipas_boeck.mat');
wps_original = tmp.guias{20};
wpDist = 3.0;
wps = resample_waypoints(wps_original, wpDist);

npts_seg = 200;
[sx, sy] = avaliar_bspline(wps, npts_seg);

dx = diff(sx);  dy = diff(sy);
ds_raw = sqrt(dx.^2 + dy.^2);
s_raw  = [0; cumsum(ds_raw)];
s_mid  = s_raw(1:end-1) + ds_raw/2;
alpha_raw = unwrap(atan2(dy, dx));

%% Varredura
dt = 0.01;
res = struct('vx', {}, 'e_mean', {}, 'e_rms', {}, 'e_max', {}, ...
    'K_psi', {}, 'K_r', {});

for k = 1:n_vx
    vx    = vx_table(k);
    K_psi = Gains_Curva(k, 1);
    K_r   = Gains_Curva(k, 2);

    % alpha(t) para este vx
    t_alpha = s_mid / vx;
    t = (0:dt:t_alpha(end))';
    alpha_ref = interp1(t_alpha, alpha_raw, t, 'pchip');
    kappa = gradient(alpha_ref, dt) / vx;
    r_ref = vx * kappa;

    % Lineariza no ponto de equilibrio para este vx
    Xe = zeros(7, 1);
    Ue = [0; vx];
    [A, B] = linearizar_veiculo(Xe, Ue, p);
    Ar = A(3:7, 3:7);
    Br = B(3:7, 1);

    K_fb = [K_psi, K_r, 0, 0, 0];
    A_mf = Ar - Br * K_fb;
    B_alpha = Br * K_psi;
    B_rref  = Br * K_r;
    C_psi   = [1 0 0 0 0];

    sys_2in = ss(A_mf, [B_alpha, B_rref], C_psi, [0 0]);

    % CI em regime
    x0 = -A_mf \ (B_alpha * alpha_ref(1));

    % Simula com FF
    [psi_out, ~] = lsim(sys_2in, [alpha_ref, r_ref], t, x0);

    e_psi = alpha_ref - psi_out;
    e_deg = rad2deg(e_psi);

    res(k).vx     = vx;
    res(k).K_psi  = K_psi;
    res(k).K_r    = K_r;
    res(k).e_mean = mean(abs(e_deg));
    res(k).e_rms  = rms(e_deg);
    res(k).e_max  = max(abs(e_deg));

    fprintf('  vx=%.1f  K=[%.1f, %.1f]  e_mean=%.2f  e_rms=%.2f  e_max=%.2f\n', ...
        vx, K_psi, K_r, res(k).e_mean, res(k).e_rms, res(k).e_max);
end

%% Tabela resumo
fprintf('\n============================================\n');
fprintf('  RESUMO — Tracking alpha com feedforward\n');
fprintf('============================================\n');
fprintf('%5s %7s %7s %8s %8s %8s\n', 'vx', 'K_psi', 'K_r', 'e_mean(deg)', 'e_rms(deg)', 'e_max(deg)');
for k = 1:n_vx
    fprintf('%5.1f %7.1f %7.1f %7.2f %7.2f %7.2f\n', ...
        res(k).vx, res(k).K_psi, res(k).K_r, ...
        res(k).e_mean, res(k).e_rms, res(k).e_max);
end

%% Plot
figure('Name', 'Tracking alpha vs vx');

subplot(2,1,1);
plot([res.vx], [res.e_mean], 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b', ...
    'DisplayName', 'mean |e_\psi|'); hold on;
plot([res.vx], [res.e_rms], 'rs-', 'LineWidth', 1.5, 'MarkerFaceColor', 'r', ...
    'DisplayName', 'rms e_\psi');
plot([res.vx], [res.e_max], 'k^-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k', ...
    'DisplayName', 'max |e_\psi|');
grid on; ylabel('e_\psi [deg]'); xlabel('v_x [m/s]');
legend('Location', 'best');
title('Erro de tracking \alpha(t) com feedforward');

subplot(2,1,2);
plot([res.vx], [res.K_psi], 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b', ...
    'DisplayName', 'K_\psi'); hold on;
plot([res.vx], [res.K_r], 'rs-', 'LineWidth', 1.5, 'MarkerFaceColor', 'r', ...
    'DisplayName', 'K_r');
grid on; ylabel('Ganho'); xlabel('v_x [m/s]');
legend('Location', 'best');
title('Ganhos otimizados');

sgtitle('Varredura tracking \alpha — taipa real, com feedforward');
