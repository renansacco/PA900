clear; close all;

load(fullfile(fileparts(mfilename('fullpath')), 'logs', 'PA900_TESTE_sess3_14h15m45s.mat'), 'data');

t      = data.time(:);
course = data.GNSS.course(:);     % graus, compass (0=N, CW+)
yaw    = data.IMU.yaw(:);         % graus, compass
gz     = data.IMU.gyro_z(:);      % deg/s, sinal invertido em relacao ao yaw
speed  = data.GNSS.speed(:);
gstat  = data.GNSS.status(:);

% --- bias do gyro_z com veiculo parado nos primeiros segundos ---
T_STILL   = 6.0;     % s
V_STILL   = 0.3;     % m/s
nInit     = find(t <= T_STILL, 1, 'last');
idxStill  = find(speed(1:nInit) < V_STILL);
if numel(idxStill) < 10, idxStill = 1:min(50,nInit); end
gz_bias   = mean(gz(idxStill));
fprintf('bias gyro_z: %+.5f deg/s  (n=%d amostras, t<%.1fs, speed<%.1f)\n', ...
        gz_bias, numel(idxStill), T_STILL, V_STILL);

% --- condicao inicial da integral ---
% usa o primeiro instante em que o veiculo ja esta em movimento com fix bom,
% ancorando em course (referencia absoluta) e nao em yaw(1) que pode estar
% afetado por ruido/alinhamento com veiculo parado.
V_OK      = 1.0;   % m/s (tambem usado na mascara de metricas abaixo)
iAnchor   = find(speed > V_OK & gstat > 0, 1, 'first');
if isempty(iAnchor), iAnchor = 1; end
PSI0      = course(iAnchor);
fprintf('ancora da integral: i=%d, t=%.2fs, PSI0=course(i)=%.2f deg\n', ...
        iAnchor, t(iAnchor), PSI0);

% --- integra -gyro_z em graus, aplica offset para psi_g(iAnchor) = PSI0 ---
gz_c   = -(gz - gz_bias);
psi_raw = [0; cumsum(0.5*(gz_c(2:end)+gz_c(1:end-1)).*diff(t))];
psi_g   = psi_raw + (PSI0 - psi_raw(iAnchor));

% --- erros (wrap a [-180,180]), NAO fazer unwrap do course ---
wrap180       = @(a) mod(a + 180, 360) - 180;
err_yaw_crs   = wrap180(yaw   - course);
err_gyro_crs  = wrap180(psi_g - course);
err_gyro_yaw  = wrap180(psi_g - yaw);

% --- mascara para metricas: velocidade suficiente + fix valido ---
mask   = speed > V_OK & gstat > 0;

fprintf('\nmascara: speed>%.1f m/s & status>0  (n=%d / %d)\n', V_OK, sum(mask), numel(mask));
fprintf('                   mean[deg]   std[deg]   max|.|[deg]\n');
errs   = {err_yaw_crs,  err_gyro_crs,  err_gyro_yaw};
labels = {'yaw   - course', 'intgy - course', 'intgy - yaw   '};
for k = 1:numel(errs)
    e = errs{k}; em = e(mask);
    fprintf('%s : %+9.2f  %9.2f  %11.2f\n', labels{k}, mean(em), std(em), max(abs(em)));
end

% deriva residual (bias remanescente do gyro)
p = polyfit(t, err_gyro_yaw, 1);
fprintf('deriva (intgy-yaw): %+.5f deg/s  (bias residual)\n', p(1));

% saltos de 180 graus no course (informativo)
dc = wrap180(diff(course));
idxJump = find(abs(dc) > 30);
if ~isempty(idxJump)
    fprintf('\nsaltos de course >30deg: %d ocorrencias\n', numel(idxJump));
    for k = 1:numel(idxJump)
        i = idxJump(k);
        fprintf('  t=%.2fs: course %.1f -> %.1f  (speed=%.2f m/s)\n', ...
                t(i+1), course(i), course(i+1), speed(i+1));
    end
end

% --- plots ---
figure('Name','Coerencia heading (course vs yaw vs int gyro_z)','Color','w');

% para a visualizacao: unwrap yaw (continuo), psi_g ja e continuo (integral),
% e ancorar cada amostra de course ao multiplo de 360 mais proximo de yaw
% (evita saltos espurios em baixa velocidade sem distorcer o dado).
yaw_plot    = unwrap(yaw*pi/180)*180/pi;
psi_g_plot  = psi_g;  % integral raw, ja continua
course_plot = yaw_plot + wrap180(course - yaw_plot);
% esconde course em trechos onde ele nao eh confiavel (baixa velocidade/fix ruim)
course_plot(~mask) = NaN;

subplot(3,1,1);
plot(t, course_plot, '.', 'MarkerSize', 4); hold on; grid on;
plot(t, yaw_plot,    'LineWidth', 1.2);
plot(t, psi_g_plot,  'LineWidth', 1.0);
ylabel('heading [deg]');
legend('GNSS course','IMU yaw','\int(-gyro_z) ancorado','Location','best');
title('Headings (alinhados por amostra ao yaw; sem saltos de \pm360)');

% nos erros contra course, esconde amostras fora da mascara
err_yaw_crs_plot  = err_yaw_crs;  err_yaw_crs_plot(~mask)  = NaN;
err_gyro_crs_plot = err_gyro_crs; err_gyro_crs_plot(~mask) = NaN;

subplot(3,1,2);
plot(t, err_yaw_crs_plot,  'LineWidth', 1); hold on; grid on;
plot(t, err_gyro_crs_plot, 'LineWidth', 1);
plot(t, err_gyro_yaw,      'LineWidth', 1);
yline(0,'k:');
legend('yaw - course','\intgyro - course','\intgyro - yaw','Location','best');
ylabel('erro [deg]'); title(sprintf('Erros angulares (metricas so em speed>%.1f m/s)', V_OK));

subplot(3,1,3);
yyaxis left;  plot(t, speed, 'LineWidth', 1); ylabel('speed [m/s]');
yyaxis right; plot(t, gstat, 'LineWidth', 1); ylabel('GNSS status');
grid on; xlabel('t [s]');
title('Contexto: velocidade e status GNSS');
