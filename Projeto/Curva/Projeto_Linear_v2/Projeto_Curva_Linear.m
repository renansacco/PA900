%% Projeto do controlador linear de curva — caso unico
%
% Otimiza ganhos [K_psi, K_r] para um vx e Delta especificos.
% Config carregada de config_curva.m.

clear; close all;

%% Paths
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));

%% Planta e config
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));
[cfg, gamma0_design, T_look] = config_curva();

%% Caso a simular
vx        = 2.0;
cfg.Delta = vx * T_look;
cfg.e0    = abs(gamma0_design) * cfg.Delta;
gamma0    = -cfg.e0 / cfg.Delta;
cfg.Q_psi = 1.0 / (gamma0)^2;

fprintf('=== Configuracao ===\n');
fprintf('  vx=%.1f m/s, Delta=%.1f m (T_look=%.1fs)\n', vx, cfg.Delta, T_look);
fprintf('  gamma0=%.1f deg, tau_la=%.2f s (e0=%.1fm)\n', ...
    rad2deg(gamma0), cfg.Delta/vx, cfg.e0);
fprintf('  Q_psi=%.2f, R=%.4f, Q/R=%.1f\n', ...
    cfg.Q_psi, cfg.R_ctrl, cfg.Q_psi/cfg.R_ctrl);

%% Otimizacao
Ks0 = [30, 40];
opts = optimset('Display', 'iter', 'MaxIter', 50, 'TolX', 1e-8);

cfg.plot = false;
[Ks, Jopt] = fminsearch(@(k) objetivo_curva(k, p, vx, cfg), Ks0, opts);

fprintf('\n=== Resultado ===\n');
fprintf('  K_psi = %.4f\n', Ks(1));
fprintf('  K_r   = %.4f\n', Ks(2));
fprintf('  J     = %.6f\n', Jopt);

%% Simula resultado final (com plot)
cfg.plot = true;
[~, t, X, U] = objetivo_curva(Ks, p, vx, cfg);

%% Metricas de desempenho
tau_la = cfg.Delta / vx;
psi_ref_final = gamma0 * exp(-t / tau_la);
e_psi = psi_ref_final - X(:, 3);

info.ts       = settling_time(t, e_psi, 0.02 * abs(gamma0));
info.Mp       = max(0, max(e_psi)) / abs(e_psi(1));
info.emax     = max(abs(e_psi));
info.umax     = max(abs(U(:,1)));
info.usat_pct = 100 * sum(abs(U(:,1)) >= cfg.omega_sat * 0.99) / length(t);

fprintf('\n=== Desempenho ===\n');
fprintf('  Settling time (2%%): %.2f s  (|e_psi| < %.1f deg)\n', ...
    info.ts, rad2deg(0.02 * abs(gamma0)));
fprintf('  Overshoot:          %.1f%%  (psi ultrapassa 0)\n', info.Mp * 100);
fprintf('  e_psi max:          %.2f deg\n', rad2deg(info.emax));
fprintf('  omega_m max:        %.1f rad/s\n', info.umax);
fprintf('  Saturacao:          %.1f%% do tempo\n', info.usat_pct);

%% Analise da malha fechada (linearizada)
analise_malha_fechada(p, vx, Ks, cfg);

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

    C_psi = [1 0 0 0 0];

    K_fb = [K_psi, K_r, 0, 0, 0];
    A_mf = Ar - Br * K_fb;
    B_mf = Br * K_psi;
    C_mf = C_psi;

    sys_T = ss(A_mf, B_mf, C_mf, 0);
    sys_L = ss(Ar, Br, K_fb, 0);
    sys_S = ss(A_mf, B_mf, -C_mf, 1);

    poles = eig(A_mf);
    [~, isort] = sort(abs(real(poles)));
    poles = poles(isort);
    pole_dom = poles(1);

    w_vec = logspace(-2, 2, 2000);
    [mag_T, phase_T] = bode(sys_T, w_vec);
    mag_T_dB = 20*log10(squeeze(mag_T));
    mag0 = mag_T_dB(1);
    idx_bw = find(mag_T_dB < mag0 - 3, 1, 'first');
    if ~isempty(idx_bw)
        w_bw = w_vec(idx_bw);
    else
        w_bw = w_vec(end);
    end
    tau_mf = 1 / w_bw;

    Mt = max(squeeze(mag_T));
    Mt_dB = 20*log10(Mt);

    [mag_S, ~] = bode(sys_S, w_vec);
    mag_S_dB = 20*log10(squeeze(mag_S));
    Ms = max(squeeze(mag_S));
    Ms_dB = 20*log10(Ms);

    dc_gain = dcgain(sys_T);
    [Gm, Pm, Wcg, Wcp] = margin(sys_L);
    omega_la = vx / cfg.Delta;

    mf.Pm = Pm;
    mf.Gm = Gm;
    mf.w_bw = w_bw;
    mf.tau_mf = tau_mf;
    mf.Mt = Mt;
    mf.Ms = Ms;

    fprintf('\n=== Malha Fechada (linearizada, vx=%.1f m/s) ===\n', vx);
    fprintf('  Ganho DC:            %.4f  (ideal=1)\n', dc_gain);
    fprintf('  Bandwidth (-3dB):    %.2f rad/s  (f=%.2f Hz)\n', w_bw, w_bw/(2*pi));
    fprintf('  tau_MF = 1/w_bw:     %.2f s\n', tau_mf);
    fprintf('  tau_la = Delta/vx:   %.2f s\n', tau_la);
    fprintf('  tau_MF / tau_la:     %.2f  (< 1 = ok)\n', tau_mf / tau_la);
    fprintf('  omega_la / omega_bw: %.2f  (< 0.5 = confortavel)\n', omega_la / w_bw);
    fprintf('  Pico |T| (Mt):      %.2f dB  (%.2f lin)\n', Mt_dB, Mt);
    fprintf('  Pico |S| (Ms):      %.2f dB  (%.2f lin)\n', Ms_dB, Ms);
    fprintf('  Polo dominante:      %.2f %+.2fj  (wn=%.2f, zeta=%.2f)\n', ...
        real(pole_dom), imag(pole_dom), abs(pole_dom), -real(pole_dom)/abs(pole_dom));
    fprintf('  Margem de ganho:     %.1f dB  (em %.2f rad/s)\n', 20*log10(Gm), Wcg);
    fprintf('  Margem de fase:      %.1f deg  (em %.2f rad/s)\n', Pm, Wcp);
    fprintf('  Polos MF:           ');
    for i = 1:length(poles)
        if imag(poles(i)) >= 0
            if imag(poles(i)) == 0
                fprintf(' %.2f', real(poles(i)));
            else
                fprintf(' %.2f%+.2fj', real(poles(i)), imag(poles(i)));
            end
        end
    end
    fprintf('\n');

    figure('Name', sprintf('Malha Fechada vx=%.1f', vx));

    subplot(2,2,1); hold on;
    semilogx(w_vec, mag_T_dB, 'b', 'LineWidth', 1.5, 'DisplayName', 'T(s)');
    semilogx(w_vec, mag_S_dB, 'r', 'LineWidth', 1.5, 'DisplayName', 'S(s)');
    xline(w_bw, 'b:', sprintf('\\omega_{bw}=%.1f', w_bw), 'HandleVisibility', 'off');
    xline(omega_la, 'k--', sprintf('\\omega_{la}=%.2f', omega_la), 'HandleVisibility', 'off');
    yline(-3, 'b:', '-3dB', 'HandleVisibility', 'off');
    yline(0, 'k-', 'HandleVisibility', 'off');
    ylabel('Magnitude [dB]'); grid on;
    legend('Location', 'best');
    title('T(s) = MF,  S(s) = Sensibilidade');
    set(gca, 'XScale', 'log');

    subplot(2,2,3); hold on;
    semilogx(w_vec, squeeze(phase_T), 'b', 'LineWidth', 1.5);
    xline(w_bw, 'b:', 'HandleVisibility', 'off');
    xline(omega_la, 'k--', 'HandleVisibility', 'off');
    ylabel('Fase T(s) [deg]'); xlabel('\omega [rad/s]'); grid on;
    set(gca, 'XScale', 'log');

    subplot(2,2,2);
    [y_step, t_step] = step(sys_T, 0:0.01:10);
    plot(t_step, y_step, 'b', 'LineWidth', 1.5); hold on;
    yline(1, 'k--'); yline(1.02, 'r:'); yline(0.98, 'r:');
    ylabel('\psi / \psi_{ref}'); xlabel('t [s]'); grid on;
    title('Step response (linearizado)');

    subplot(2,2,4);
    plot(real(poles), imag(poles), 'bx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
    plot(real(eig(Ar)), imag(eig(Ar)), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
    xline(0, 'k'); yline(0, 'k'); grid on;
    xlabel('Re'); ylabel('Im');
    legend('MF', 'MA', 'Location', 'best');
    title('Polos');

    sgtitle(sprintf('Malha Fechada — K_{\\psi}=%.1f, K_r=%.1f, v_x=%.1f m/s', ...
        K_psi, K_r, vx));
end
