% plot_sim.m — Plot dos resultados da simulacao closed-loop
%
% Requer no workspace: out (SimulationOutput), wps (struct com .x, .y)

%% Extrai sinais
close all;
t = 0:0.01:Tsim;
e           = out.e.signals.values;
psi_deg     = out.psi_deg.signals.values;
psi_ref_deg = out.psi_ref_deg.signals.values;
r           = out.r.signals.values;
omegam      = out.omegam.signals.values;
omegam_ref  = out.omegam_ref.signals.values;
delta_deg   = out.delta_deg.signals.values;
curvature   = out.curvature.signals.values;
vx_sig      = out.vx.signals.values;
xy          = out.r_IC.signals.values;

% Heading continuo (unwrap para evitar saltos de 360)
psi_cont     = rad2deg(unwrap(deg2rad(psi_deg)));
psi_ref_cont = rad2deg(unwrap(deg2rad(psi_ref_deg)));

%% Figura 1 — Trajetoria
figure('Name', 'Trajetoria');
plot(wps.x, wps.y, 'k--', 'DisplayName', 'Referencia'); hold on;
plot(xy(:,1), xy(:,2), 'b', 'LineWidth', 1.5, 'DisplayName', 'Veiculo');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'best');
title('Trajetoria');

%% Figura 2 — Controle (3x2 subplots)
figure('Name', 'Controle');

subplot(3,2,1);
plot(t, e); grid on;
ylabel('e [m]');
title('Erro lateral');

subplot(3,2,2);
plot(t, psi_cont, t, psi_ref_cont, '--'); grid on;
ylabel('[deg]');
legend('\psi', '\psi_{ref}', 'Location', 'best');
title('Heading');

subplot(3,2,3);
plot(t, psi_cont - psi_ref_cont); grid on;
ylabel('[deg]');
title('Erro angular (\psi - \alpha)');

subplot(3,2,4);
plot(t, r); grid on;
ylabel('r [rad/s]');
title('Yaw rate');

subplot(3,2,5);
plot(t, omegam, t, omegam_ref, '--'); grid on;
ylabel('[rad/s]');
legend('\omega_m', '\omega_{m,ref}', 'Location', 'best');
title('Velocidade angular motor');
xlabel('Tempo [s]');

subplot(3,2,6);
plot(t, delta_deg); grid on;
ylabel('\delta [deg]');
title('Angulo de ester�amento');
xlabel('Tempo [s]');

%% Figura 3 — Contexto da trajetoria
figure('Name', 'Contexto');

subplot(2,1,1);
plot(out.curvature.time, curvature); grid on;
ylabel('\kappa [1/m]');
title('Curvatura');

subplot(2,1,2);
plot(out.vx.time, vx_sig); grid on;
ylabel('v_x [m/s]');
title('Velocidade longitudinal');
xlabel('Tempo [s]');
