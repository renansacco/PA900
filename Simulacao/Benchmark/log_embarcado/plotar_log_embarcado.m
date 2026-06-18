function plotar_log_embarcado(logDir)
% PLOTAR_LOG_EMBARCADO  Plota dados do log embarcado/simulador para comparacao com benchmark
%
% Uso:
%   plotar_log_embarcado('D:\MG900_Export\Fazendas\TYT\A1\TESTE2')
%
% Gera plots no mesmo layout do plotar_cenario.m para comparacao direta.

if nargin < 1
    logDir = 'D:\MG900_Export\Fazendas\TYT\A1\TESTE2';
end

%% Paths
thisDir = fileparts(mfilename('fullpath'));
addpath(fullfile(thisDir, '..', 'trajetorias'));

%% Parametros do veiculo
k_d = 0.0437;   % gear ratio motor->delta
Lr  = 1.0;
try
    p = load('param_MF6713.mat');
    k_d = p.k_d;
    Lr  = p.Lr;
catch
end

%% Gera waypoints da guia (replica pipeline embarcado: shapefile 2.0m -> resample 3.0m)
R = 6; len_reta = 30; wpDist_shp = 2.0; wpDist_emb = 3.0;
n1 = round(len_reta / wpDist_shp);
x1 = linspace(0, len_reta, n1+1)';  y1 = zeros(size(x1));
n2 = round(pi*R / wpDist_shp);
th = linspace(0, pi, n2+1)'; th = th(2:end);
x2 = len_reta + R*sin(th);  y2 = R*(1-cos(th));
n3 = round(len_reta / wpDist_shp);
x3 = linspace(len_reta, 0, n3+1)'; x3 = x3(2:end);  y3 = 2*R*ones(size(x3));
wps_shp.x = [x1; x2; x3];  wps_shp.y = [y1; y2; y3];
wps_guide = resample_waypoints(wps_shp, wpDist_emb);
fprintf('Guia: %d wps originais -> %d wps reamostrados (%.2fm)\n', ...
    numel(wps_shp.x), numel(wps_guide.x), wpDist_emb);

%% Encontra e le o CSV de log
logFiles = dir(fullfile(logDir, 'log', 'logger_*.csv'));
if isempty(logFiles)
    error('Nenhum arquivo logger_*.csv encontrado em %s/log/', logDir);
end
[~, imax] = max([logFiles.bytes]);
csvFile = fullfile(logFiles(imax).folder, logFiles(imax).name);
T = readtable(csvFile);
fprintf('Log: %s (%d amostras)\n', logFiles(imax).name, height(T));

%% Filtra periodo com autopiloto ON
mask_ap = T.isAutopilotOn == 1;
if ~any(mask_ap)
    error('Nenhuma amostra com isAutopilotOn=1');
end
T_ap = T(mask_ap, :);
fprintf('Autopiloto ON: %d amostras (%.1f s)\n', height(T_ap), ...
    (T_ap.tickMs(end) - T_ap.tickMs(1)) / 1000);

%% Tempo relativo [s]
t = (T_ap.tickMs - T_ap.tickMs(1)) / 1000;

%% Converte lat/lon para ENU (metros) — origem do shapefile exportado
origin_lat = -29.0;
origin_lon = -53.0;
try
    wgs84 = wgs84Ellipsoid('meter');
    [xe, yn, ~] = geodetic2enu(T_ap.lat, T_ap.lon, zeros(height(T_ap),1), ...
        origin_lat, origin_lon, 0, wgs84);
catch
    cos_lat = cosd(origin_lat);
    xe = (T_ap.lon - origin_lon) * cos_lat * 111320;
    yn = (T_ap.lat - origin_lat) * 111320;
end

%% Sinais de controle
e_lat       = T_ap.lateralError;
psi_error   = T_ap.psiError;
yaw_deg     = T_ap.yaw;
course_deg  = T_ap.course;
alpha_rad   = T_ap.alpha;
curvature   = T_ap.curvature;
speed_kmh   = T_ap.speed;
vx          = speed_kmh / 3.6;
gyro_z      = T_ap.gyro_z;           % deg/s
omega_m     = T_ap.steerSpeedMeasured;
omega_m_ref = T_ap.steerSpeedTarget;
theta_rad   = T_ap.thetaRad;
delta_deg   = rad2deg(k_d * theta_rad);

%% Converte angulos para graus (convencao matematica)
% yaw/course sao compass (0=N, CW) -> math (0=E, CCW)
heading_math_deg = 90 - yaw_deg;
course_math_deg  = 90 - course_deg;
alpha_deg        = rad2deg(alpha_rad);
psi_error_deg    = rad2deg(psi_error);

% Unwrap para continuidade
heading_cont = rad2deg(unwrap(deg2rad(heading_math_deg)));
course_cont  = rad2deg(unwrap(deg2rad(course_math_deg)));
alpha_cont   = rad2deg(unwrap(alpha_rad));

%% Estimativa de sideslip
gyro_z_rad = deg2rad(gyro_z);
vx_safe = max(vx, 0.5);
beta_gyro = atan(Lr * gyro_z_rad ./ vx_safe);
beta_kappa = atan(Lr * curvature);

%% Avalia B-spline nos waypoints da guia (o que o guidance produz)
[bsp_guide_x, bsp_guide_y] = avaliar_bspline(wps_guide, 30);

%% Identifica regioes (approach / reta / curva / reta)
kappa_thresh = 0.02;
in_curve = abs(curvature) > kappa_thresh;

%% ============================================================
%%  FIGURA 1 — Trajetoria
%% ============================================================
titulo = sprintf('Log embarcado — %s', logDir);

figure('Name', 'Trajetoria — Log embarcado');
plot(wps_shp.x, wps_shp.y, 'k.--', 'DisplayName', 'Shapefile (2.0m)'); hold on;
plot(wps_guide.x, wps_guide.y, 'ms', 'MarkerSize', 5, ...
    'DisplayName', sprintf('Guia reamostrada (%.1fm)', wpDist_emb));
plot(bsp_guide_x, bsp_guide_y, 'r-', 'LineWidth', 1.5, 'DisplayName', 'B-spline (guidance)');
plot(xe, yn, 'b', 'LineWidth', 1, 'DisplayName', 'Veiculo (GPS)');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'best');
title(titulo, 'Interpreter', 'none');

%% ============================================================
%%  FIGURA 2 — Sinais de controle (5x2, mesmo layout do plotar_cenario)
%% ============================================================
figure('Name', sprintf('Controle — Log embarcado'));
sgtitle(titulo, 'Interpreter', 'none');

% (1,1) Erro lateral
subplot(5,2,1);
plot(t, e_lat*100); grid on;
ylabel('e [cm]');
title('Erro lateral');

% (1,2) Heading / Course / Alpha
subplot(5,2,2);
plot(t, heading_cont, 'DisplayName', 'heading'); hold on;
plot(t, course_cont, ':', 'LineWidth', 1.5, 'DisplayName', 'course');
plot(t, alpha_cont, '--', 'DisplayName', 'alpha');
grid on;
ylabel('[deg]');
legend('Location', 'best');
title('Heading / Course / Alpha');

% (2,1) Erro angular
subplot(5,2,3);
plot(t, psi_error_deg, 'DisplayName', 'psiError'); grid on;
ylabel('[deg]');
legend('Location', 'best');
title('Erro angular (heading)');

% (2,2) Yaw rate vs kappa*vx
subplot(5,2,4);
plot(t, gyro_z, 'DisplayName', 'gyro_z'); hold on;
plot(t, rad2deg(curvature .* vx), '--', 'DisplayName', 'kappa*vx');
grid on;
ylabel('[deg/s]');
legend('Location', 'best');
title('Yaw rate vs referencia');

% (3,1) omega_m vs omega_m_ref
subplot(5,2,5);
plot(t, omega_m, 'DisplayName', 'omega_m'); hold on;
plot(t, omega_m_ref, '--', 'DisplayName', 'omega_{m,ref}');
grid on;
ylabel('[rad/s]');
legend('Location', 'best');
title('Velocidade angular motor');

% (3,2) Delta (angulo esterco)
subplot(5,2,6);
plot(t, delta_deg); grid on;
ylabel('[deg]');
title(sprintf('Angulo de estercamento (k_d=%.4f)', k_d));

% (4,1) Curvatura
subplot(5,2,7);
plot(t, curvature); grid on;
ylabel('\kappa [1/m]');
title('Curvatura');

% (4,2) Velocidade
subplot(5,2,8);
plot(t, speed_kmh); grid on;
ylabel('[km/h]');
title('Velocidade');

% (5,1) Sideslip estimado
subplot(5,2,9);
plot(t, rad2deg(beta_gyro), 'DisplayName', 'atan(Lr*r/vx)'); hold on;
plot(t, rad2deg(beta_kappa), '--', 'DisplayName', 'atan(Lr*kappa)');
grid on;
ylabel('[deg]');
legend('Location', 'best');
title('Sideslip estimado');

% (5,2) Servo current
subplot(5,2,10);
plot(t, T_ap.servoCurrent); grid on;
ylabel('[A]');
title('Corrente servo');

%% ============================================================
%%  FIGURA 3 — Metricas resumo
%% ============================================================
% Descarta primeiros 2s (approach)
t_descarte = 2.0;
mask_ss = t > t_descarte;
e_ss = e_lat(mask_ss);

figure('Name', 'Metricas — Log embarcado');
sgtitle(titulo, 'Interpreter', 'none');

subplot(1,2,1);
histogram(e_ss*100, 30, 'FaceColor', [0.3 0.6 0.9]);
grid on;
xlabel('e_{lat} [cm]');
ylabel('count');
title(sprintf('Histograma (mean=%.1fcm, max=%.1fcm, rms=%.1fcm)', ...
    mean(abs(e_ss))*100, max(abs(e_ss))*100, rms(e_ss)*100));

% Metricas por regiao
subplot(1,2,2);
e_reta = e_lat(mask_ss & ~in_curve);
e_curva = e_lat(mask_ss & in_curve);
labels = {'Reta: mean'; 'Reta: max'; 'Reta: rms'; ...
          'Curva: mean'; 'Curva: max'; 'Curva: rms'; ...
          'Total: mean'; 'Total: max'; 'Total: rms'};
vals = [mean(abs(e_reta))*100; max(abs(e_reta))*100; rms(e_reta)*100; ...
        mean(abs(e_curva))*100; max(abs(e_curva))*100; rms(e_curva)*100; ...
        mean(abs(e_ss))*100; max(abs(e_ss))*100; rms(e_ss)*100];
barh(vals, 'FaceColor', [0.4 0.7 0.5]);
set(gca, 'YTick', 1:numel(labels), 'YTickLabel', labels);
grid on;
xlabel('[cm]');
title('Erro lateral por regiao');

%% Imprime metricas
fprintf('\n=== Metricas (t > %.1fs) ===\n', t_descarte);
fprintf('  TOTAL:  e_mean=%.1fcm  e_max=%.1fcm  e_rms=%.1fcm\n', ...
    mean(abs(e_ss))*100, max(abs(e_ss))*100, rms(e_ss)*100);
if any(~in_curve & mask_ss)
    fprintf('  RETA:   e_mean=%.1fcm  e_max=%.1fcm  e_rms=%.1fcm\n', ...
        mean(abs(e_reta))*100, max(abs(e_reta))*100, rms(e_reta)*100);
end
if any(in_curve & mask_ss)
    fprintf('  CURVA:  e_mean=%.1fcm  e_max=%.1fcm  e_rms=%.1fcm\n', ...
        mean(abs(e_curva))*100, max(abs(e_curva))*100, rms(e_curva)*100);
    fprintf('  Curvatura media na curva: kappa=%.4f (R=%.1fm)\n', ...
        mean(abs(curvature(in_curve))), 1/mean(abs(curvature(in_curve))));
end
fprintf('  Velocidade media: %.1f km/h (%.2f m/s)\n', ...
    mean(speed_kmh), mean(vx));
fprintf('  Sideslip medio (curva): %.2f deg (gyro), %.2f deg (kappa)\n', ...
    rad2deg(mean(abs(beta_gyro(in_curve)))), ...
    rad2deg(mean(abs(beta_kappa(in_curve)))));
fprintf('  delta max: %.1f deg\n', max(abs(delta_deg)));
fprintf('  omega_m max: %.1f rad/s\n', max(abs(omega_m)));
fprintf('  Duracao autopiloto: %.1f s\n', t(end));

end
