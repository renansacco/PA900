%% Simula tracking de alpha(t) pela malha fechada linearizada
%
% Monta o sistema de 2 entradas (alpha, r_ref) a partir da planta
% linearizada + ganhos K_psi, K_r. Alimenta com alpha(t) e vx*kappa(t).
%
% Lei de controle: u = K_psi*(alpha - psi) + K_r*(r_ref - r)
% r_ref = vx * kappa = d(alpha)/dt

clear; close all;

%% Paths
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));

%% === PARAMETROS ===
vx = 3.0;

% Ganhos do controlador (extrair de Projeto_Curva_Linear)
K_psi = 27.0;
K_r   = 36.0;

%% Lineariza planta no ponto de equilibrio
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));

Xe = zeros(7, 1);
Ue = [0; vx];
[A, B] = linearizar_veiculo(Xe, Ue, p);

% Estados reduzidos: [psi, r, vy, omega_m, delta]
Ar = A(3:7, 3:7);
Br = B(3:7, 1);

% Malha fechada: feedback K_fb = [K_psi, K_r, 0, 0, 0]
K_fb = [K_psi, K_r, 0, 0, 0];
A_mf = Ar - Br * K_fb;

% Duas entradas de referencia:
%   alpha entra via K_psi,  r_ref entra via K_r
B_alpha = Br * K_psi;
B_rref  = Br * K_r;

C_psi = [1 0 0 0 0];

% Sistema 2-entrada 1-saida: [alpha; r_ref] -> psi
sys_2in = ss(A_mf, [B_alpha, B_rref], C_psi, [0 0]);

% Sistema so feedback (sem FF): alpha -> psi
sys_noff = ss(A_mf, B_alpha, C_psi, 0);

poles_mf = eig(A_mf);
[~, isort] = sort(abs(real(poles_mf)));
poles_mf = poles_mf(isort);

fprintf('=== Malha Fechada (vx=%.1f) ===\n', vx);
fprintf('  K_psi=%.1f, K_r=%.1f\n', K_psi, K_r);
fprintf('  Polos MF: ');
for i = 1:length(poles_mf)
    if imag(poles_mf(i)) >= 0
        if imag(poles_mf(i)) == 0
            fprintf('%.2f  ', real(poles_mf(i)));
        else
            fprintf('%.2f%+.2fj  ', real(poles_mf(i)), imag(poles_mf(i)));
        end
    end
end
fprintf('\n');
fprintf('  DC gain (alpha->psi): %.4f\n', dcgain(sys_noff));

%% Trajetoria: alpha(t) da taipa real
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

t_alpha = s_mid / vx;
dt = 0.01;
t = (0:dt:t_alpha(end))';
alpha_ref = interp1(t_alpha, alpha_raw, t, 'pchip');
kappa = gradient(alpha_ref, dt) / vx;
r_ref = vx * kappa;

fprintf('\n=== Trajetoria ===\n');
fprintf('  vx = %.1f m/s\n', vx);
fprintf('  Path = %.0f m, Tsim = %.1f s\n', s_mid(end), t(end));
fprintf('  alpha range = %.1f deg\n', rad2deg(max(alpha_ref) - min(alpha_ref)));

%% Simula: com e sem feedforward
% CI em regime com alpha(0) constante e r_ref=0
% x_ss = -A_mf \ (B_alpha * alpha(0))
x0 = -A_mf \ (B_alpha * alpha_ref(1));

% Com FF: 2 entradas [alpha, r_ref]
u_in_ff = [alpha_ref, r_ref];
[psi_ff, ~] = lsim(sys_2in, u_in_ff, t, x0);

% Sem FF: so alpha
[psi_noff, ~] = lsim(sys_noff, alpha_ref, t, x0);

%% Erros
e_ff   = alpha_ref - psi_ff;
e_noff = alpha_ref - psi_noff;

fprintf('\n=== Erro de tracking (com feedforward) ===\n');
fprintf('  e_psi mean: %.2f deg\n', mean(abs(rad2deg(e_ff))));
fprintf('  e_psi rms:  %.2f deg\n', rms(rad2deg(e_ff)));
fprintf('  e_psi max:  %.2f deg\n', max(abs(rad2deg(e_ff))));

fprintf('\n=== Erro de tracking (sem feedforward) ===\n');
fprintf('  e_psi mean: %.2f deg\n', mean(abs(rad2deg(e_noff))));
fprintf('  e_psi rms:  %.2f deg\n', rms(rad2deg(e_noff)));
fprintf('  e_psi max:  %.2f deg\n', max(abs(rad2deg(e_noff))));

%% === Plots ===
figure('Name', sprintf('Tracking alpha — vx=%.1f m/s', vx));

subplot(3,1,1);
plot(t, rad2deg(alpha_ref), 'r', 'LineWidth', 1.5, 'DisplayName', '\alpha_{ref}'); hold on;
plot(t, rad2deg(psi_ff), 'b', 'DisplayName', 'com FF');
plot(t, rad2deg(psi_noff), 'g--', 'DisplayName', 'sem FF');
grid on; ylabel('[deg]');
legend('Location', 'best');
title(sprintf('Tracking de \\alpha(t) — vx=%.1f m/s', vx));

subplot(3,1,2);
plot(t, rad2deg(e_ff), 'b', 'DisplayName', ...
    sprintf('com FF (rms=%.2f°)', rms(rad2deg(e_ff)))); hold on;
plot(t, rad2deg(e_noff), 'g--', 'DisplayName', ...
    sprintf('sem FF (rms=%.2f°)', rms(rad2deg(e_noff))));
grid on; ylabel('e_\psi [deg]');
legend('Location', 'best');
title('Erro de tracking');

subplot(3,1,3);
plot(t, kappa, 'b');
grid on; ylabel('\kappa [1/m]'); xlabel('t [s]');
title('Curvatura do path');

sgtitle(sprintf('K_{\\psi}=%.0f, K_r=%.0f, vx=%.1f m/s', K_psi, K_r, vx));
