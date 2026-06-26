function [J, tout, Xout, Uout] = objetivo_curva(Ks, p, vx, cfg)
% Funcao de custo para otimizacao dos ganhos do controlador de curva.
%
%   Ks  = [K_psi, K_r]
%   p   = struct com parametros da planta (param_MF6713.mat)
%   vx  = velocidade longitudinal [m/s]
%   cfg = struct com:
%     e0, Delta, tsim, omega_sat, X0,
%     Q_psi, Q_r, R_ctrl, plot
%
% Referencia: gamma(t) = -(e0/Delta)*exp(-vx*t/Delta)
% Derivada da dinamica do lookahead linearizada (tau = Delta/vx).

K_psi = Ks(1); K_r = Ks(2);

%% Referencia: exponencial do lookahead
tau_la = cfg.Delta / vx;
gamma0 = -cfg.e0 / cfg.Delta;

%% Simula malha fechada
odefun = @(t, X) malha_fechada(t, X, p, vx, K_psi, K_r, gamma0, tau_la, cfg.omega_sat);
[tout, Xout] = ode45(odefun, [0 cfg.tsim], cfg.X0);

%% Reconstroi referencia e entrada de controle
n = length(tout);
Uout    = zeros(n, 2);
psi_ref = zeros(n, 1);
for i = 1:n
    psi_ref(i) = gamma0 * exp(-tout(i) / tau_la);
    e_psi = psi_ref(i) - Xout(i, 3);
    r_i   = Xout(i, 4);
    u_psi = K_psi * e_psi + K_r * (0 - r_i);
    u_psi = max(-cfg.omega_sat, min(cfg.omega_sat, u_psi));
    Uout(i, :) = [u_psi, vx];
end

%% Custo quadratico
psi   = Xout(:, 3);
e_psi = psi_ref - psi;

J_psi = trapz(tout, tout .* cfg.Q_psi .* e_psi.^2);
J_r   = trapz(tout, 1 .* cfg.Q_r .* Xout(:,4).^2);
J_u   = trapz(tout, cfg.R_ctrl .* Uout(:,1).^2);
J_base = J_psi + J_r + J_u;

%% Restricoes de MF
J_pm = 0;
J_bw = 0;
Pm = NaN;
tau_mf_ratio = NaN;

Xe = zeros(7, 1);
Ue = [0; vx];
[A, B] = linearizar_veiculo(Xe, Ue, p);
Ar = A(3:7, 3:7);
Br = B(3:7, 1);
K_fb = [K_psi, K_r, 0, 0, 0];

% Margem de fase
sys_L = ss(Ar, Br, K_fb, 0);
[~, Pm] = margin(sys_L);
if isfield(cfg, 'Pm_min') && cfg.Pm_min > 0 && Pm < cfg.Pm_min
    J_pm = J_base * 100 * ((cfg.Pm_min - Pm) / cfg.Pm_min)^2;
end

% Bandwidth: tau_MF < tau_la * cfg.tau_ratio_max
if isfield(cfg, 'tau_ratio_max') && cfg.tau_ratio_max > 0
    A_mf = Ar - Br * K_fb;
    B_mf = Br * K_psi;
    C_mf = [1 0 0 0 0];
    sys_T = ss(A_mf, B_mf, C_mf, 0);
    w_vec = logspace(-2, 2, 500);
    [mag_bw, ~] = bode(sys_T, w_vec);
    mag_dB = 20*log10(squeeze(mag_bw));
    idx_bw = find(mag_dB < mag_dB(1) - 3, 1, 'first');
    if ~isempty(idx_bw)
        w_bw = w_vec(idx_bw);
    else
        w_bw = w_vec(end);
    end
    tau_mf = 1 / w_bw;
    tau_mf_ratio = tau_mf / tau_la;
    if tau_mf_ratio > cfg.tau_ratio_max
        J_bw = J_base * 100 * ((tau_mf_ratio - cfg.tau_ratio_max) / cfg.tau_ratio_max)^2;
    end
end

J = J_base + J_pm + J_bw;

%% Decomposicao do custo (ultima iteracao com plot)
if cfg.plot
    fprintf('\n=== Decomposicao do custo ===\n');
    fprintf('  J_psi (ITAE heading): %.4f  (%.0f%%)\n', J_psi, 100*J_psi/J);
    fprintf('  J_r   (ITAE r):     %.4f  (%.0f%%)\n', J_r,   100*J_r/J);
    fprintf('  J_u   (controle):     %.4f  (%.0f%%)\n', J_u,   100*J_u/J);
    fprintf('  J_base:               %.4f\n', J_base);
    fprintf('  J_pm  (fase):         %.4f  (Pm=%.1f deg)\n', J_pm, Pm);
    fprintf('  J_bw  (bandwidth):    %.4f  (tau_MF/tau_la=%.2f)\n', J_bw, tau_mf_ratio);
    fprintf('  J_total:              %.4f\n', J);
end

%% Plot durante otimizacao (opcional)
if cfg.plot
    figure; clf;

    subplot(3,2,1);
    plot(tout, rad2deg(psi), tout, rad2deg(psi_ref), 'r--');
    ylabel('\psi [deg]'); grid on;
    legend('\psi', '\psi_{ref}', 'Location', 'best');
    title(sprintf('Heading  (\\tau_{la}=%.2fs, \\gamma_0=%.1f°)', tau_la, rad2deg(gamma0)));

    subplot(3,2,2);
    plot(tout, rad2deg(e_psi));
    ylabel('e_\psi [deg]'); grid on;
    title('Erro de heading');

    subplot(3,2,3);
    plot(tout, Xout(:,7), 'DisplayName', '\omega_m'); hold on;
    plot(tout, Uout(:,1), 'r--', 'DisplayName', '\omega_{m,ref}');
    yline(cfg.omega_sat, 'k:', 'HandleVisibility', 'off');
    yline(-cfg.omega_sat, 'k:', 'HandleVisibility', 'off');
    ylabel('[rad/s]'); grid on;
    legend('Location', 'best');
    title('Velocidade motor');

    subplot(3,2,4);
    plot(tout, rad2deg(Xout(:,6)));
    ylabel('\delta [deg]'); grid on;
    title('Angulo de esterco');

    subplot(3,2,5);
    plot(tout, rad2deg(Xout(:,4)));
    ylabel('r [deg/s]'); grid on;
    xlabel('t [s]');
    title('Yaw rate');

    subplot(3,2,6);
    plot(tout, Xout(:,5));
    ylabel('v_y [m/s]'); grid on;
    xlabel('t [s]');
    title('Velocidade lateral');

    sgtitle(sprintf('K=[%.1f, %.1f]  J=%.4f  vx=%.1f  e_0=%.1fm  \\Delta=%.1fm', ...
        K_psi, K_r, J, vx, cfg.e0, cfg.Delta));
    drawnow;
end

end

%% -----------------------------------------------------------------------
function Xp = malha_fechada(t, X, p, vx, K_psi, K_r, gamma0, tau_la, omega_sat)
    psi = X(3); r = X(4);

    psi_ref = gamma0 * exp(-t / tau_la);

    omega_m_ref = K_psi * (psi_ref - psi) + K_r * (0 - r);
    omega_m_ref = max(-omega_sat, min(omega_sat, omega_m_ref));

    Xp = dinamica_veiculo(X, [omega_m_ref; vx], p);
end
