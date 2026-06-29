%% Analise: porque o sistema oscila em alta velocidade?
%
% Compara a planta linearizada e a MF para varias velocidades,
% mantendo o mesmo tau_mf/tau_la (mesma banda passante relativa).
% Mostra o que muda na planta e porque o controle fica mais dificil.

clear; close all;

%% Setup
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));

cfg = config_curva();
gamma0_design = cfg.gamma0;
T_look = cfg.T_look;

vx_list = [1, 2, 3, 4];
colors = lines(length(vx_list));
w_vec = logspace(-2, 2, 2000);

%% Carrega ganhos otimizados
gains_file = fullfile(fileparts(mfilename('fullpath')), '..', 'ERT', 'gains', 'Curva_Gains_Linear.mat');
gdata = load(gains_file);

%% ================================================================
%% FIGURA 1 — Planta em malha aberta: como muda com vx
%% ================================================================
figure('Name', 'Planta vs vx');

for k = 1:length(vx_list)
    vx = vx_list(k);
    Xe = zeros(7,1); Ue = [0; vx];
    [A, B] = linearizar_veiculo(Xe, Ue, p);
    Ar = A(3:7, 3:7);
    Br = B(3:7, 1);

    % G_r: omega_m -> r (yaw rate)
    C_r = [0 1 0 0 0];
    sys_Gr = ss(Ar, Br, C_r, 0);

    % G_psi: omega_m -> psi
    C_psi = [1 0 0 0 0];
    sys_Gpsi = ss(Ar, Br, C_psi, 0);

    % Polos da planta
    poles_ma = eig(Ar);

    % Bode G_r
    [mag_r, phase_r] = bode(sys_Gr, w_vec);

    subplot(2,2,1); hold on;
    semilogx(w_vec, 20*log10(squeeze(mag_r)), 'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('vx=%.0f', vx));
    subplot(2,2,3); hold on;
    semilogx(w_vec, squeeze(phase_r), 'Color', colors(k,:), 'LineWidth', 1.5);

    % Polos
    subplot(2,2,2); hold on;
    plot(real(poles_ma), imag(poles_ma), 'o', 'Color', colors(k,:), ...
        'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', sprintf('vx=%.0f', vx));

    % Ganho DC e damping
    poles_lat = poles_ma(abs(imag(poles_ma)) > 0.01);
    if ~isempty(poles_lat)
        p_dom = poles_lat(1);
        wn = abs(p_dom);
        zeta = -real(p_dom) / wn;
    else
        [~, idx_dom] = min(abs(real(poles_ma)));
        p_dom = poles_ma(idx_dom);
        wn = abs(p_dom);
        zeta = 1.0;
    end
    subplot(2,2,4); hold on;
    plot(vx, zeta, 'o', 'Color', colors(k,:), 'MarkerSize', 10, ...
        'LineWidth', 2, 'MarkerFaceColor', colors(k,:));
    text(vx+0.1, zeta, sprintf('\\zeta=%.2f\n\\omega_n=%.1f', zeta, wn), 'FontSize', 9);
end

subplot(2,2,1);
ylabel('|G_r| [dB]'); grid on; set(gca,'XScale','log');
legend('Location','best'); title('Planta MA: \omega_m \rightarrow r');
subplot(2,2,3);
ylabel('Fase [deg]'); xlabel('\omega [rad/s]'); grid on; set(gca,'XScale','log');
subplot(2,2,2);
xline(0,'k'); yline(0,'k'); grid on;
xlabel('Re'); ylabel('Im'); title('Polos da planta (MA)');
legend('Location','best');
subplot(2,2,4);
ylabel('\zeta (amortecimento)'); xlabel('v_x [m/s]'); grid on;
title('Amortecimento do polo lateral');
sgtitle('Como a planta muda com v_x');

%% ================================================================
%% FIGURA 2 — Malha fechada com ganhos otimizados (mesmo tau_ratio)
%% ================================================================
figure('Name', 'MF vs vx');

summary = struct();

for k = 1:length(vx_list)
    vx = vx_list(k);
    Delta = vx * T_look;
    tau_la = Delta / vx;

    % Interpola ganhos da tabela
    K_psi = interp1(gdata.vx_table, gdata.Gains_Curva(:,1), vx, 'linear');
    K_r   = interp1(gdata.vx_table, gdata.Gains_Curva(:,2), vx, 'linear');

    Xe = zeros(7,1); Ue = [0; vx];
    [A, B] = linearizar_veiculo(Xe, Ue, p);
    Ar = A(3:7, 3:7);
    Br = B(3:7, 1);

    K_fb = [K_psi, K_r, 0, 0, 0];
    A_mf = Ar - Br * K_fb;

    sys_T = ss(A_mf, Br*K_psi, [1 0 0 0 0], 0);
    sys_L = ss(Ar, Br, K_fb, 0);
    sys_S = ss(A_mf, Br*K_psi, -[1 0 0 0 0], 1);

    [mag_T, ~] = bode(sys_T, w_vec);
    mag_T_dB = 20*log10(squeeze(mag_T));
    [mag_S, ~] = bode(sys_S, w_vec);
    mag_S_dB = 20*log10(squeeze(mag_S));

    [~, Pm] = margin(sys_L);
    Ms = max(squeeze(mag_S));

    poles_mf = eig(A_mf);

    % Bandwidth
    idx_bw = find(mag_T_dB < mag_T_dB(1) - 3, 1, 'first');
    if ~isempty(idx_bw), w_bw = w_vec(idx_bw); else, w_bw = w_vec(end); end

    summary(k).vx = vx;
    summary(k).Pm = Pm;
    summary(k).Ms = Ms;
    summary(k).w_bw = w_bw;
    summary(k).K_psi = K_psi;
    summary(k).K_r = K_r;

    % |T(jw)|
    subplot(2,2,1); hold on;
    semilogx(w_vec, mag_T_dB, 'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('vx=%.0f (Pm=%.0f°)', vx, Pm));

    % |S(jw)| — pico = sensibilidade a perturbacao
    subplot(2,2,2); hold on;
    semilogx(w_vec, mag_S_dB, 'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('vx=%.0f (Ms=%.1f)', vx, Ms));

    % Polos MF
    subplot(2,2,3); hold on;
    plot(real(poles_mf), imag(poles_mf), 'x', 'Color', colors(k,:), ...
        'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', sprintf('vx=%.0f', vx));

    % Step response
    subplot(2,2,4); hold on;
    [y, t] = step(sys_T, 0:0.01:8);
    plot(t, y, 'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('vx=%.0f', vx));
end

subplot(2,2,1);
yline(-3, 'k:', '-3dB'); yline(0, 'k-');
ylabel('|T| [dB]'); grid on; set(gca,'XScale','log');
legend('Location','best'); title('Malha fechada T(s)');
subplot(2,2,2);
yline(6, 'r:', 'Ms=6dB (perigoso)');
ylabel('|S| [dB]'); grid on; set(gca,'XScale','log');
legend('Location','best'); title('Sensibilidade S(s) — pico = oscilacao');
subplot(2,2,3);
xline(0,'k'); yline(0,'k'); grid on;
xlabel('Re'); ylabel('Im');
legend('Location','best'); title('Polos MF');
subplot(2,2,4);
yline(1,'k--'); yline(1.02,'r:'); yline(0.98,'r:');
ylabel('\psi/\psi_{ref}'); xlabel('t [s]'); grid on;
legend('Location','best'); title('Step response');
sgtitle('Malha fechada — mesmo \tau_{MF}/\tau_{la}, diferentes v_x');

%% ================================================================
%% FIGURA 3 — Resumo: Pm, Ms, zeta do polo dominante MF vs vx
%% ================================================================
figure('Name', 'Metricas vs vx');

vx_fine = 0.5:0.25:4.0;
Pm_vec = zeros(size(vx_fine));
Ms_vec = zeros(size(vx_fine));
zeta_mf_vec = zeros(size(vx_fine));
wn_mf_vec = zeros(size(vx_fine));
zeta_ma_vec = zeros(size(vx_fine));

for k = 1:length(vx_fine)
    vx = vx_fine(k);
    K_psi = interp1(gdata.vx_table, gdata.Gains_Curva(:,1), vx, 'linear', 'extrap');
    K_r   = interp1(gdata.vx_table, gdata.Gains_Curva(:,2), vx, 'linear', 'extrap');

    Xe = zeros(7,1); Ue = [0; vx];
    [A, B] = linearizar_veiculo(Xe, Ue, p);
    Ar = A(3:7, 3:7);
    Br = B(3:7, 1);

    K_fb = [K_psi, K_r, 0, 0, 0];
    A_mf = Ar - Br * K_fb;

    sys_L = ss(Ar, Br, K_fb, 0);
    sys_S = ss(A_mf, Br*K_psi, -[1 0 0 0 0], 1);

    [~, Pm_vec(k)] = margin(sys_L);
    [mag_S, ~] = bode(sys_S, w_vec);
    Ms_vec(k) = max(squeeze(mag_S));

    % Polo dominante MF (menor |Re|)
    poles_mf = eig(A_mf);
    [~, isort] = sort(abs(real(poles_mf)));
    p_dom = poles_mf(isort(1));
    wn_mf_vec(k) = abs(p_dom);
    zeta_mf_vec(k) = -real(p_dom) / abs(p_dom);

    % Polo lateral MA
    poles_ma = eig(Ar);
    poles_lat = poles_ma(abs(imag(poles_ma)) > 0.01);
    if ~isempty(poles_lat)
        zeta_ma_vec(k) = -real(poles_lat(1)) / abs(poles_lat(1));
    else
        [~, idx] = min(abs(real(poles_ma)));
        zeta_ma_vec(k) = 1.0;
    end
end

subplot(2,2,1);
plot(vx_fine, Pm_vec, 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
yline(cfg.Pm_min, 'r--', sprintf('Pm_{min}=%d°', cfg.Pm_min));
ylabel('Pm [deg]'); xlabel('v_x [m/s]'); grid on;
title('Margem de fase');

subplot(2,2,2);
plot(vx_fine, 20*log10(Ms_vec), 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
yline(6, 'r--', 'Ms=6dB (oscilatorio)');
ylabel('Ms [dB]'); xlabel('v_x [m/s]'); grid on;
title('Pico de sensibilidade (quanto maior, mais oscila)');

subplot(2,2,3);
plot(vx_fine, zeta_ma_vec, 'ro-', 'LineWidth', 1.5, 'MarkerFaceColor', 'r', ...
    'DisplayName', 'Planta (MA)'); hold on;
plot(vx_fine, zeta_mf_vec, 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b', ...
    'DisplayName', 'Malha fechada');
ylabel('\zeta'); xlabel('v_x [m/s]'); grid on;
legend('Location','best');
title('Amortecimento do polo dominante');

subplot(2,2,4);
plot(vx_fine, wn_mf_vec, 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
ylabel('\omega_n [rad/s]'); xlabel('v_x [m/s]'); grid on;
title('Frequencia natural do polo dominante MF');

sgtitle('Porque o sistema oscila em alta velocidade');

%% Console
fprintf('\n=== Resumo: efeito de vx na planta e MF ===\n');
fprintf('%5s %6s %6s %6s %8s %8s\n', 'vx', 'K_psi', 'K_r', 'Pm', 'Ms[dB]', 'zeta_MF');
for k = 1:length(vx_list)
    idx = find(abs(vx_fine - vx_list(k)) < 0.01, 1);
    fprintf('%5.1f %6.1f %6.1f %6.1f %8.1f %8.2f\n', ...
        vx_fine(idx), ...
        interp1(gdata.vx_table, gdata.Gains_Curva(:,1), vx_list(k)), ...
        interp1(gdata.vx_table, gdata.Gains_Curva(:,2), vx_list(k)), ...
        Pm_vec(idx), 20*log10(Ms_vec(idx)), zeta_mf_vec(idx));
end
