%% Analise espectral de alpha(s) — angulo da trajetoria B-spline
%
% O espectro espacial de alpha eh propriedade da geometria do path.
% Quando percorrido a velocidade vx, freq espacial nu [1/m] mapeia
% para freq temporal: f = nu * vx [Hz],  omega = 2*pi*nu*vx [rad/s].
%
% Inclui analise local (spectrograma) para dimensionar pelo pior caso.

clear; close all;

%% Paths
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));

%% Trajetoria (taipa real, resampled como o embarcado)
tmp = load('taipas_boeck.mat');
wps_original = tmp.guias{20};
wpDist = 3.0;
wps = resample_waypoints(wps_original, wpDist);

%% B-spline com alta resolucao
npts_seg = 200;
[sx, sy] = avaliar_bspline(wps, npts_seg);

%% alpha(s) uniforme em arclength
dx = diff(sx);  dy = diff(sy);
ds_raw = sqrt(dx.^2 + dy.^2);
s_raw  = [0; cumsum(ds_raw)];
s_mid  = s_raw(1:end-1) + ds_raw/2;
alpha_raw = unwrap(atan2(dy, dx));

ds_uni = 0.01;  % reamostra a 1 cm
s_uni = (0:ds_uni:s_mid(end))';
alpha_uni = interp1(s_mid, alpha_raw, s_uni, 'pchip');
kappa_uni = gradient(alpha_uni, ds_uni);
N_s = length(alpha_uni);

%% ===== Figura 1: Dominio espacial =====
figure('Name', 'Dominio Espacial');

subplot(3,1,1);
plot(wps.x, wps.y, 'k.--'); hold on;
plot(sx, sy, 'b', 'LineWidth', 1.2);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trajetoria B-spline (taipa real)');
legend('Waypoints', 'B-spline', 'Location', 'best');

subplot(3,1,2);
plot(s_uni, rad2deg(alpha_uni), 'b', 'LineWidth', 1.2);
grid on; xlabel('s [m]'); ylabel('\alpha [deg]');
title('\alpha(s) — heading ao longo do path');

subplot(3,1,3);
plot(s_uni, kappa_uni, 'b', 'LineWidth', 1.2);
grid on; xlabel('s [m]'); ylabel('\kappa [1/m]');
title('\kappa(s) = d\alpha/ds');

sgtitle('Trajetoria B-spline — dominio espacial');

%% ===== FFT espacial de alpha(s) — computado uma unica vez =====
alpha_det = detrend(alpha_uni);
win = hanning(N_s);
Nfft = 2^nextpow2(N_s);
Y = fft(alpha_det .* win, Nfft);
nu_vec = (0:Nfft/2) / (Nfft * ds_uni);   % freq espacial [cycles/m]
mag_alpha = abs(Y(1:Nfft/2+1)) * 2 / sum(win);

% Energia acumulada no dominio espacial
psd = abs(Y(1:Nfft/2+1)).^2;
energia_total = sum(psd);
energia_acum = cumsum(psd) / energia_total * 100;

idx95 = find(energia_acum >= 95, 1, 'first');
idx99 = find(energia_acum >= 99, 1, 'first');
nu95 = nu_vec(idx95);
nu99 = nu_vec(idx99);

%% ===== Figura 2: Espectro espacial + mapeamento temporal =====
velocidades = [1.0, 2.0, 3.0, 4.0];
colors_v = lines(length(velocidades));

figure('Name', 'Espectro de alpha(s)');

% --- Espectro espacial (eixo em nu) ---
subplot(2,2,1);
plot(nu_vec, 20*log10(mag_alpha + eps), 'k', 'LineWidth', 1.2);
xline(nu95, 'r--', sprintf('\\nu_{95}=%.3f', nu95));
xline(nu99, 'b:', sprintf('\\nu_{99}=%.3f', nu99));
grid on; xlabel('\nu [cycles/m]'); ylabel('|\alpha(\nu)| [dB]');
title('Espectro espacial de \alpha(s)');
xlim([0, max(0.5, nu99*2)]);

% --- Mesmo espectro, eixo mapeado para omega por vx ---
subplot(2,2,2);
for iv = 1:length(velocidades)
    vx = velocidades(iv);
    omega_mapped = 2*pi * nu_vec * vx;
    semilogx(omega_mapped, 20*log10(mag_alpha + eps), ...
        'Color', colors_v(iv,:), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('vx=%.1f m/s', vx)); hold on;
end
grid on; xlabel('\omega [rad/s]'); ylabel('|\alpha(\omega)| [dB]');
title('Mesmo espectro, eixo: \omega = 2\pi\nu v_x');
legend('Location', 'northeast');
xlim([0.01, 50]);

% --- Energia acumulada (eixo nu) ---
subplot(2,2,3);
plot(nu_vec, energia_acum, 'k', 'LineWidth', 1.5);
xline(nu95, 'r--', '95%');
xline(nu99, 'b:', '99%');
yline(95, 'r:'); yline(99, 'b:');
grid on; xlabel('\nu [cycles/m]'); ylabel('Energia acumulada [%]');
title('Energia acumulada — dominio espacial');
xlim([0, max(0.5, nu99*2)]); ylim([0 105]);

% --- Energia acumulada (eixo omega por vx) ---
subplot(2,2,4);
for iv = 1:length(velocidades)
    vx = velocidades(iv);
    omega_mapped = 2*pi * nu_vec * vx;
    semilogx(omega_mapped, energia_acum, ...
        'Color', colors_v(iv,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('vx=%.1f m/s', vx)); hold on;
end
yline(95, 'r:'); yline(99, 'b:');
grid on; xlabel('\omega [rad/s]'); ylabel('Energia acumulada [%]');
title('Energia acumulada — mapeada para \omega');
legend('Location', 'southeast');
xlim([0.01, 50]); ylim([0 105]);

sgtitle('Espectro espacial de \alpha(s) — FFT unico, rescalado por v_x');

%% ===== Figura 3: Spectrograma — analise local (nao-estacionariedade) =====
win_length_m = 100;  % janela de 30 m
win_samples = round(win_length_m / ds_uni);
if mod(win_samples, 2) == 1, win_samples = win_samples + 1; end
noverlap = round(win_samples * 0.75);
nfft_local = 2^nextpow2(win_samples);

figure('Name', 'Spectrograma de alpha(s)');

subplot(3,1,1);
plot(s_uni, rad2deg(alpha_uni), 'b'); hold on;
yyaxis right;
plot(s_uni, kappa_uni, 'r');
ylabel('\kappa [1/m]');
yyaxis left;
ylabel('\alpha [deg]');
xlabel('s [m]'); grid on;
title('Contexto: \alpha(s) e \kappa(s)');

subplot(3,1,2);
fs_spatial = 1 / ds_uni;
hop = win_samples - noverlap;
n_windows = floor((N_s - win_samples) / hop) + 1;
F_spec = (0:nfft_local/2)' * fs_spatial / nfft_local;  % [cycles/m]
T_spec = zeros(1, n_windows);
S_spec = zeros(length(F_spec), n_windows);
w = hanning(win_samples);

for iw = 1:n_windows
    i0 = (iw-1)*hop + 1;
    seg = alpha_uni(i0:i0+win_samples-1);
    seg = detrend(seg);
    Yw = fft(seg .* w, nfft_local);
    S_spec(:, iw) = abs(Yw(1:nfft_local/2+1));
    T_spec(iw) = s_uni(i0) + win_length_m/2;
end

imagesc(T_spec, F_spec, 20*log10(S_spec + eps));
axis xy; colorbar;
ylabel('\nu [cycles/m]'); xlabel('s [m]');
ylim([0, max(0.5, nu99*3)]);
title(sprintf('Spectrograma de \\alpha(s) — janela %.0f m, overlap 75%%, detrend local', win_length_m));

subplot(3,1,3);
nu95_local = zeros(1, n_windows);
for iw = 1:n_windows
    psd_local = S_spec(:, iw).^2;
    e_acum = cumsum(psd_local) / sum(psd_local) * 100;
    idx = find(e_acum >= 95, 1, 'first');
    if ~isempty(idx)
        nu95_local(iw) = F_spec(idx);
    end
end
plot(T_spec, nu95_local, 'b', 'LineWidth', 1.5);
grid on; xlabel('s [m]'); ylabel('\nu_{95} local [cycles/m]');
yline(nu95, 'k:', sprintf('\\nu_{95} global = %.3f', nu95));
[nu95_max, idx_max] = max(nu95_local);
xline(T_spec(idx_max), 'r--', sprintf('pior caso s=%.0fm', T_spec(idx_max)));
title('\nu_{95} local — freq espacial contendo 95% da energia por janela');

sgtitle('Analise de nao-estacionariedade');


%% FIG 4

%% ===== Borda de banda: espectro absoluto vs piso de ruido =====
% Le a BORDA (joelho onde o conteudo estruturado encontra o piso) � mais
% robusta que o percentil nu95, que e sensivel a normalizacao.

nu   = nu_vec(:);
P_dB = 10*log10(psd(:)/max(psd) + eps);     % PSD global, rel. ao pico [dB]
P_s  = movmean(P_dB, 15);                   % suaviza p/ achar o joelho

% Piso de ruido: mediana acima de 0.2 cycles/m (alem de qualquer geometria real)
floor_dB = median(P_dB(nu >= 0.20));
margin_dB = 10;                             % "real" = > piso + margem

% Borda = maior nu (abaixo de 0.3) ainda acima de piso+margem
above   = find(P_s > floor_dB + margin_dB & nu < 0.30);
nu_edge = nu(above(end));

% Tetos da fonte de dados (waypoints a wpDist)
nu_wp_nyq = 1/(2*wpDist);                   % Nyquist do waypoint
nu_wp_rel = 1/(3*wpDist);                   % confiavel (~3 wp/feicao)

% Nivel especifico em nu=0.1 (o pico do espectrograma)
lvl_01 = interp1(nu, P_dB, 0.1) - floor_dB;

figure('Name','Borda de banda � espectro absoluto');
semilogx(nu, P_dB, 'Color',[.65 .65 .65]); hold on;
semilogx(nu, P_s, 'b', 'LineWidth',1.5);
yline(floor_dB, 'k:', 'piso de ruido');
yline(floor_dB+margin_dB, 'k--', sprintf('piso + %d dB', margin_dB));
xline(nu_edge,  'r-',  sprintf('borda \\nu=%.3f', nu_edge), 'LineWidth',1.2);
xline(0.1,      'm--', '\nu=0.1');
xline(nu_wp_rel,'g--', sprintf('lim. waypoints %.3f', nu_wp_rel));
xline(nu_wp_nyq,'g:',  sprintf('Nyquist wp %.3f', nu_wp_nyq));
grid on; xlabel('\nu [cycles/m]'); ylabel('PSD [dB rel. pico]');
title('Espectro global de \alpha � borda vs piso de ruido');
xlim([nu(2), 0.5]); ylim([floor_dB-10, 5]);

fprintf('\n=== Borda de banda (espectro absoluto) ===\n');
fprintf('  piso de ruido:     %.1f dB (rel. pico)\n', floor_dB);
fprintf('  borda (piso+%ddB):  nu = %.4f cycles/m (lambda = %.1f m)\n', ...
        margin_dB, nu_edge, 1/nu_edge);
fprintf('  nivel em nu=0.1:   %.1f dB acima do piso\n', lvl_01);
fprintf('  teto waypoints:    nu = %.4f (lambda = %.1f m)\n', nu_wp_rel, 1/nu_wp_rel);
if lvl_01 > margin_dB
    fprintf('  -> conteudo em 0.1 e REAL (%.1f dB acima do piso)\n', lvl_01);
else
    fprintf('  -> 0.1 esta ~no piso (%.1f dB) -> artefato de resolucao\n', lvl_01);
end

%% ===== Resumo numerico =====
fprintf('\n=== Trajetoria ===\n');
fprintf('  Path length:     %.1f m\n', s_uni(end));
fprintf('  Waypoint dist:   %.1f m\n', wpDist);
fprintf('  N waypoints:     %d\n', numel(wps.x));
fprintf('  kappa max:       %.4f 1/m\n', max(abs(kappa_uni)));
fprintf('  kappa mean |k|:  %.4f 1/m\n', mean(abs(kappa_uni)));
fprintf('  alpha range:     %.1f deg\n', rad2deg(max(alpha_uni) - min(alpha_uni)));

fprintf('\n=== Espectro espacial de alpha — global ===\n');
fprintf('  nu_95:  %.4f cycles/m  (lambda_95 = %.1f m)\n', nu95, 1/nu95);
fprintf('  nu_99:  %.4f cycles/m  (lambda_99 = %.1f m)\n', nu99, 1/nu99);

fprintf('\n=== Pior caso local (nu_95 max na janela de %.0f m) ===\n', win_length_m);
fprintf('  nu_95 max:  %.4f cycles/m  em s = %.0f m\n', nu95_max, T_spec(idx_max));

fprintf('\n=== nu_95 global → frequencia temporal por velocidade ===\n');
fprintf('  %6s  %12s  %12s\n', 'vx', 'f_95 [Hz]', 'omega_95');
for iv = 1:length(velocidades)
    vx = velocidades(iv);
    fprintf('  %5.1f   %11.3f   %11.2f\n', vx, vx*nu95, 2*pi*vx*nu95);
end

fprintf('\n=== nu_95 pior caso → frequencia temporal por velocidade ===\n');
fprintf('  %6s  %12s  %12s\n', 'vx', 'f_95 [Hz]', 'omega_95');
for iv = 1:length(velocidades)
    vx = velocidades(iv);
    fprintf('  %5.1f   %11.3f   %11.2f\n', vx, vx*nu95_max, 2*pi*vx*nu95_max);
end
