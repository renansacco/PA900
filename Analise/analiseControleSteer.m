clear; close all;

load(fullfile(fileparts(mfilename('fullpath')), 'logs', 'PA900_TESTE_sess3_14h15m45s.mat'), 'data');

% --- constantes ---
k    = 0.05;    % motor -> roda (delta = k * thetaRad)  [= volante->roda, pois motor e direto no eixo do volante]
L    = 2.5;     % entre-eixos [m]
V_OK = 1.0;     % m/s

% --- sinais ---
t      = data.time(:);
sst    = data.Control.steerSpeedTarget(:);
ssm    = data.Control.steerSpeedMeasured(:);
theta  = data.Control.thetaRad(:);
ap     = data.Control.isAutopilotOn(:) > 0;
gz     = data.IMU.gyro_z(:);       % deg/s
speed  = data.GNSS.speed(:) / 3.6; % GNSS.speed vem em km/h -> converte para m/s

% --- integral trapezoidal de steerSpeedMeasured, ancorada em theta(1) ---
theta_int = theta(1) + [0; cumsum(0.5*(ssm(2:end)+ssm(1:end-1)).*diff(t))];

% --- modelo cinematico de bicicleta ---
delta     = k * theta;                              % rad na roda
gz_pred   = speed .* tan(delta) / L * 180/pi;       % deg/s

% --- mascara de validade ---
mask = speed > V_OK & ap;
fprintf('k=%.2f, L=%.2f, V_OK=%.1f\n', k, L, V_OK);
fprintf('amostras autopilot on: %d / %d\n', sum(ap), numel(ap));
fprintf('amostras mascara (speed>V_OK & autopilot): %d / %d\n', sum(mask), numel(mask));

% --- deteccao de inversao de sinal ---
if sum(mask) > 10
    c = corrcoef(gz(mask), gz_pred(mask));
    corr_val = c(1,2);
else
    corr_val = NaN;
end
fprintf('correlacao gyro_z vs modelo (mascara): %+.2f\n', corr_val);
if ~isnan(corr_val) && corr_val < 0
    gz_pred = -gz_pred;
    fprintf('AVISO: sinal invertido detectado entre gyro_z e modelo. Aplicando inversao (k -> -k).\n');
else
    fprintf('sinal coerente (sem inversao).\n');
end

% --- plots ---
gz_pred_plot = gz_pred;       gz_pred_plot(~mask) = NaN;
residuo      = gz - gz_pred;  residuo(~mask)      = NaN;

fig = figure('Name','Analise controle de direcao (steerSpeed / thetaRad / gyro_z)','Color','w');

ax1 = subplot(4,1,1);
plot(t, sst, 'LineWidth', 1); hold on; grid on;
plot(t, ssm, 'LineWidth', 1);
legend('target','measured','Location','best');
ylabel('steerSpeed [rad/s]');
title('Malha interna do motor');

ax2 = subplot(4,1,2);
plot(t, theta,     'LineWidth', 1); hold on; grid on;
plot(t, theta_int, 'LineWidth', 1);
legend('thetaRad medido','\int steerSpeedMeasured','Location','best');
ylabel('thetaRad [rad]');
title('Consistencia encoder vs tacometro');

ax3 = subplot(4,1,3);
plot(t, gz,           'LineWidth', 1); hold on; grid on;
plot(t, gz_pred_plot, 'LineWidth', 1);
legend('gyro_z medido','modelo bicicleta','Location','best');
ylabel('gyro_z [deg/s]');
title('gyro_z medido vs predito (mask: speed>V\_OK & autopilot)');

ax4 = subplot(4,1,4);
yyaxis left;  plot(t, residuo, 'LineWidth', 1); ylabel('residuo [deg/s]'); grid on;
yyaxis right; plot(t, speed,   'LineWidth', 1); ylabel('speed [m/s]');
xlabel('t [s]');
title('Residuo (gyro_z - predito) e velocidade');

linkaxes([ax1 ax2 ax3 ax4], 'x');

% =====================================================================
% Integridade delta (atuador) vs gyro_z (IMU): scatter + cross-corr
% =====================================================================
gm = gz(mask);
pm = gz_pred(mask);

% --- (1) scatter com reta y=x e ajuste linear (informativo) ---
p = polyfit(pm, gm, 1);   % slope, intercept
lim = max(abs([gm; pm])) * 1.05;
xx  = linspace(-lim, lim, 100);

figure('Name','Integridade delta vs gyro_z','Color','w');
subplot(1,2,1);
plot(pm, gm, '.', 'MarkerSize', 6); hold on; grid on;
plot(xx, xx,          'k--', 'LineWidth', 1);          % y = x
plot(xx, p(1)*xx+p(2),'r-',  'LineWidth', 1.2);        % ajuste
axis equal; xlim([-lim lim]); ylim([-lim lim]);
xlabel('modelo (v/L) tan(\delta)  [deg/s]');
ylabel('gyro_z medido [deg/s]');
title(sprintf('Scatter (n=%d)   slope=%.3f  intercept=%+.2f deg/s', numel(gm), p(1), p(2)));
legend('amostras','y = x','ajuste OLS','Location','best');

fprintf('\n[Integridade] scatter: slope=%.3f, intercept=%+.2f deg/s\n', p(1), p(2));
fprintf('  slope~1   => escala de k*L correta\n');
fprintf('  intercept~0 => sem offset em delta\n');

% --- (2) cross-correlacao para estimar defasagem ---
% reamostra em grade uniforme (linear interp) usando dt mediano,
% so sobre a mascara principal (mesmo intervalo temporal contiguo)
Ts_u = median(diff(t));
tu   = (t(1):Ts_u:t(end))';
gz_u    = interp1(t, gz,      tu, 'linear');
pred_u  = interp1(t, gz_pred, tu, 'linear');
mask_u  = interp1(t, double(mask), tu, 'linear') > 0.5;

% centra sinais na mascara (remove media) para xcorr nao pegar bias DC
g0 = gz_u - mean(gz_u(mask_u));
p0 = pred_u - mean(pred_u(mask_u));
g0(~mask_u) = 0;
p0(~mask_u) = 0;

maxLag = round(1.5 / Ts_u);                 % +/- 1.5s
[c, lags] = xcorr(g0, p0, maxLag, 'coeff');
[cmax, imax] = max(c);
lag_samples = lags(imax);
lag_sec     = lag_samples * Ts_u;

subplot(1,2,2);
plot(lags*Ts_u, c, 'LineWidth', 1); hold on; grid on;
yline(0,'k:'); xline(lag_sec,'r--');
xlabel('lag [s]   (positivo = gyro atrasado em relacao ao modelo)');
ylabel('correlacao normalizada');
title(sprintf('xcorr   lag=%+.3f s (%+d amostras)   pico=%.3f', lag_sec, lag_samples, cmax));

fprintf('[Integridade] xcorr: lag=%+d amostras (%+.3f s), pico=%.3f\n', lag_samples, lag_sec, cmax);
fprintf('  lag~0 => sensores sincronizados\n');
fprintf('  lag>0 => gyro_z atrasado em relacao ao modelo (atuador/encoder a frente)\n');
fprintf('  lag<0 => modelo atrasado em relacao ao gyro_z\n');

