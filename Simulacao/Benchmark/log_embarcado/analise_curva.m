% analise_curva.m — Analisa comportamento na curva circular
%
% Roda simulacao com trajetoria cabeceira (reta+curva+reta),
% condicao alinhada, e analisa os sinais na regiao da curva
% para entender o erro lateral constante.
%
% Requer: benchmark_trajs.mat, modelClosedLoop carregado

clear; %close all;

%% Setup
params = load('param_MF6713.mat');
Ts_guidance = 0.05;
Param_Controller;

%% Gera trajetoria: reta de entrada + circulo completo (N voltas)
R = 6;           % raio [m]
N_voltas = 3;    % numero de voltas no circulo
wpDist = 2.0;    % espacamento entre waypoints [m]

% Reta de entrada (30m em +x)
len_reta = 30;
n_reta = round(len_reta / wpDist);
x_reta = linspace(0, len_reta, n_reta + 1)';
y_reta = zeros(size(x_reta));

% Circulo: centro em (len_reta, R), N voltas completas
n_circ = round(2*pi*R*N_voltas / wpDist);
theta = linspace(0, 2*pi*N_voltas, n_circ + 1)';
theta = theta(2:end);  % remove duplicata com fim da reta
x_circ = len_reta + R * sin(theta);
y_circ = R * (1 - cos(theta));

wps.x = [x_reta; x_circ];
wps.y = [y_reta; y_circ];

vx = 2.0;

% Condicao inicial alinhada no 2o wp
state0 = guidance_init(wps);
[g0, ~] = guidance_step([wps.x(2), wps.y(2), 0], state0);
alpha0 = g0.alpha;

X0 = zeros(7, 1);
X0(1) = wps.x(2);
X0(2) = wps.y(2);
X0(3) = alpha0;

pathLen = sum(sqrt(diff(wps.x).^2 + diff(wps.y).^2));
Tsim = ceil(pathLen / vx) + 10;

%% Roda simulacao
modelName = 'modelClosedLoop';
load_system(modelName);
out = sim(modelName);

%% Extrai sinais
t           = out.e.time;
e_lat       = out.e.signals.values;
psi         = deg2rad(out.psi_deg.signals.values);
psi_ref     = deg2rad(out.psi_ref_deg.signals.values);
r_yaw       = out.r.signals.values;
omegam_ref  = out.omegam_ref.signals.values;
delta       = deg2rad(out.delta_deg.signals.values);
curvature   = out.curvature.signals.values;
xy          = out.r_IC.signals.values;
alpha_guid  = out.alpha.signals.values;  % heading da spline

% Tenta extrair vy se disponivel
try
    vy = out.vy.signals.values;
    has_vy = true;
catch
    fprintf('AVISO: sinal vy nao encontrado no out. Adicione To Workspace no modelo.\n');
    vy = zeros(size(t));
    has_vy = false;
end

%% Parametros relevantes
L_cp = params.L_cp;
Lf   = params.Lf;
Lr   = params.Lr;
L    = Lf + Lr;
Delta_look = 4.0;  % lookahead do controller

fprintf('\n=== Parametros ===\n');
fprintf('L_cp = %.3f m (distancia CG ao ponto de controle)\n', L_cp);
fprintf('L    = %.3f m (entre-eixos)\n', L);
fprintf('Delta = %.1f m (lookahead)\n', Delta_look);
fprintf('vx   = %.1f m/s\n', vx);

%% Identifica regiao da curva (curvatura > limiar)
kappa_thresh = 0.02;  % 1/m
in_curve = abs(curvature) > kappa_thresh;

% Pega regiao central da curva (descarta transicoes)
idx_curve = find(in_curve);
if ~isempty(idx_curve)
    margin = round(0.15 * numel(idx_curve));  % descarta 15% nas bordas
    idx_curve = idx_curve(margin:end-margin);
end

fprintf('\n=== Regiao da curva (t = %.1f a %.1f s) ===\n', t(idx_curve(1)), t(idx_curve(end)));
fprintf('Curvatura media:    kappa = %.4f 1/m  (R = %.1f m)\n', ...
    mean(curvature(idx_curve)), 1/mean(abs(curvature(idx_curve))));
fprintf('Erro lateral medio: e     = %.4f m (%.1f cm)\n', ...
    mean(e_lat(idx_curve)), mean(e_lat(idx_curve))*100);
fprintf('Erro lateral sinal: %s (positivo = dentro da curva)\n', ...
    ternary(mean(e_lat(idx_curve)) > 0, 'DENTRO', 'FORA'));

%% Analise de sideslip
if has_vy
    % Sideslip no CG
    beta_cg = atan2(vy, vx);

    % Sideslip no ponto de controle
    vy_cp = vy + L_cp * r_yaw;
    beta_cp = atan2(vy_cp, vx);

    fprintf('\nSideslip medio na curva:\n');
    fprintf('  beta_CG = %.2f deg\n', rad2deg(mean(beta_cg(idx_curve))));
    fprintf('  beta_CP = %.2f deg  (vy_cp = vy + L_cp*r = %.3f + %.3f*%.3f)\n', ...
        rad2deg(mean(beta_cp(idx_curve))), ...
        mean(vy(idx_curve)), L_cp, mean(r_yaw(idx_curve)));
    fprintf('  vy medio      = %.4f m/s\n', mean(vy(idx_curve)));
    fprintf('  L_cp * r medio = %.4f m/s\n', L_cp * mean(r_yaw(idx_curve)));

    % Predicao teorica: e_ss = Delta * tan(beta_cp)
    e_predicted = Delta_look * tan(mean(beta_cp(idx_curve)));
    fprintf('\nPredicao teorica: e_ss = Delta * tan(beta_cp) = %.1f * tan(%.2f deg) = %.4f m (%.1f cm)\n', ...
        Delta_look, rad2deg(mean(beta_cp(idx_curve))), e_predicted, e_predicted*100);
    fprintf('Erro real medio:  e_ss = %.4f m (%.1f cm)\n', ...
        mean(e_lat(idx_curve)), mean(e_lat(idx_curve))*100);
end

%% Analise da referencia de heading
% O controlador faz: psi_ref = atan(-e/Delta) + alpha
% Usar wrapToPi para lidar com multiplas voltas
psi_error = wrapToPi(psi - psi_ref);
fprintf('\nErro de heading na curva:\n');
fprintf('  psi_error medio = %.3f deg\n', rad2deg(mean(psi_error(idx_curve))));
fprintf('  psi_error max   = %.3f deg\n', rad2deg(max(abs(psi_error(idx_curve)))));

% Diferenca entre heading e direcao da velocidade (alpha do guidance)
if has_vy
    psi_vs_alpha = wrapToPi(psi(idx_curve) - alpha_guid(idx_curve));
    fprintf('  psi - alpha medio   = %.3f deg (heading vs tangente spline)\n', ...
        rad2deg(mean(psi_vs_alpha)));
    vel_vs_alpha = wrapToPi(psi(idx_curve) + beta_cp(idx_curve) - alpha_guid(idx_curve));
    fprintf('  vel_dir - alpha medio = %.3f deg (direcao velocidade vs tangente)\n', ...
        rad2deg(mean(vel_vs_alpha)));
end

%% Figuras

% Fig 1: Trajetoria
figure('Name', 'Analise Curva — Trajetoria');
plot(wps.x, wps.y, 'k--', 'DisplayName', 'Referencia'); hold on;
plot(xy(:,1), xy(:,2), 'b', 'LineWidth', 1.5, 'DisplayName', 'Veiculo');
plot(xy(idx_curve,1), xy(idx_curve,2), 'r', 'LineWidth', 2, 'DisplayName', 'Regiao analisada');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'best');
title('Trajetoria — cabeceira');

% Fig 2: Sinais na curva
figure('Name', 'Analise Curva — Sinais');

subplot(3,2,1);
plot(t, e_lat*100); grid on; hold on;
xline(t(idx_curve(1)), '--r'); xline(t(idx_curve(end)), '--r');
ylabel('e_{lat} [cm]');
title('Erro lateral');

subplot(3,2,2);
plot(t, curvature); grid on; hold on;
xline(t(idx_curve(1)), '--r'); xline(t(idx_curve(end)), '--r');
ylabel('\kappa [1/m]');
title('Curvatura');

subplot(3,2,3);
plot(t, rad2deg(wrapToPi(psi - psi_ref))); grid on; hold on;
xline(t(idx_curve(1)), '--r'); xline(t(idx_curve(end)), '--r');
ylabel('[deg]');
title('Erro heading (\psi - \psi_{ref})');

subplot(3,2,4);
plot(t, r_yaw, 'DisplayName', 'r'); grid on; hold on;
plot(t, curvature*vx, '--', 'DisplayName', '\kappa v_x');
xline(t(idx_curve(1)), '--r'); xline(t(idx_curve(end)), '--r');
ylabel('[rad/s]');
legend('Location', 'best');
title('Yaw rate vs referencia');

if has_vy
    % Estimativas de beta
    beta_kinematic = atan(Lr * curvature);          % usa kappa do path
    beta_gyro      = atan(Lr * r_yaw / vx);         % usa r medido (gyro) e vx (GPS)

    subplot(3,2,5);
    plot(t, rad2deg(beta_cg), 'LineWidth', 1.5, 'DisplayName', 'real'); hold on;
    plot(t, rad2deg(beta_kinematic), '--', 'DisplayName', 'atan(Lr*kappa)');
    plot(t, rad2deg(beta_gyro), ':', 'LineWidth', 1.5, 'DisplayName', 'atan(Lr*r/vx)');
    grid on;
    xline(t(idx_curve(1)), '--r'); xline(t(idx_curve(end)), '--r');
    ylabel('[deg]');
    legend('Location', 'best');
    title('Sideslip: real vs estimativas');

    fprintf('\nEstimativas de beta na curva:\n');
    fprintf('  beta real       = %.2f deg\n', rad2deg(mean(beta_cg(idx_curve))));
    fprintf('  beta cinematico = %.2f deg  (atan(Lr*kappa))\n', ...
        rad2deg(mean(beta_kinematic(idx_curve))));
    fprintf('  beta gyro       = %.2f deg  (atan(Lr*r/vx))\n', ...
        rad2deg(mean(beta_gyro(idx_curve))));

    subplot(3,2,6);
    plot(t, vy, 'LineWidth', 1.5, 'DisplayName', 'v_y real'); hold on;
    plot(t, vx * tan(beta_kinematic), '--', 'DisplayName', 'v_y kappa');
    plot(t, Lr * r_yaw, ':', 'LineWidth', 1.5, 'DisplayName', 'Lr*r (gyro)');
    grid on;
    xline(t(idx_curve(1)), '--r'); xline(t(idx_curve(end)), '--r');
    ylabel('[m/s]');
    legend('Location', 'best');
    title('Velocidade lateral: real vs estimativas');
end

fprintf('\n=== Conclusao ===\n');
if has_vy && mean(e_lat(idx_curve)) > 0
    fprintf('Veiculo faz curva MAIS FECHADA (e > 0 = dentro da curva).\n');
    fprintf('Causa provavel: sideslip no ponto de controle (beta_cp = %.2f deg)\n', ...
        rad2deg(mean(beta_cp(idx_curve))));
    fprintf('empurra o veiculo para dentro, e o lookahead (Delta=%.0fm)\n', Delta_look);
    fprintf('nao consegue compensar totalmente.\n');
elseif has_vy
    fprintf('Veiculo faz curva MAIS ABERTA (e < 0 = fora da curva).\n');
end

function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
