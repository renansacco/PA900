% script_condicionar_guia.m — Condiciona waypoints em struct guia padronizada
%
% Fluxo: carrega .mat de waypoints → condicionar_guia → plota → salva em guias/
% Uso: ajustar a trajetoria abaixo e rodar.

clear; %close all;

%% Paths
guidanceDir = fullfile(fileparts(mfilename('fullpath')), '..');
trajDir     = fullfile(guidanceDir, 'trajetorias');

%% Carregar trajetoria
trajFile = 'taipas_boeck.mat';
trajIdx  = 20;

tmp = load(fullfile(trajDir, 'waypoints', trajFile));
wps_original = tmp.guias{trajIdx};

% wps_original = tmp.wps;

wpDist      = 1;
fc_kappa    = 0.20;   % frequencia de corte [ciclos/m]
ordem_kappa = 2;      % ordem do Butterworth

%% Construir guia densa
guia = condicionar_guia(wps_original, wpDist, 50, fc_kappa, ordem_kappa);

%% Avaliar erro dos waypoints originais
erros = erro_guia(wps_original, guia);

n_orig = numel(wps_original.x);
n_res  = numel(guia.wps.x);

fprintf('\n=== Condicionamento da Guia (wpDist=%.1f m) ===\n', wpDist);
fprintf('  Waypoints originais: %d\n', n_orig);
fprintf('  Waypoints resampled: %d\n', n_res);
fprintf('  Pontos na tabela:    %d\n', numel(guia.s));
fprintf('  s_total:             %.1f m\n', guia.s(end));
fprintf('  Erro max (wps->guia): %.3f m  (%.1f cm)\n', max(erros), max(erros)*100);
fprintf('  Erro RMS (wps->guia): %.3f m  (%.1f cm)\n', rms(erros), rms(erros)*100);
fprintf('  Erro medio:           %.3f m  (%.1f cm)\n', mean(erros), mean(erros)*100);

%% Salvar guia padronizada
[~, baseName] = fileparts(trajFile);
outFile = fullfile(trajDir, 'guias', [baseName '.mat']);
save(outFile, 'guia');
fprintf('  Guia salva em: %s\n', outFile);

%% Plots
titulo = sprintf('%s #%d | wpDist=%.1f m', trajFile, trajIdx, wpDist);

figure('Name', sprintf('Condicionamento — %s', titulo));
sgtitle(titulo, 'Interpreter', 'none');

subplot(3,2,1);
plot(wps_original.x, wps_original.y, 'ko', 'MarkerSize', 4, ...
    'DisplayName', 'WPs originais'); hold on;
plot(guia.wps.x, guia.wps.y, 'ms', 'MarkerSize', 3, ...
    'DisplayName', sprintf('Resampled (%.1fm)', wpDist));
plot(guia.x, guia.y, 'b-', 'LineWidth', 1.2, ...
    'DisplayName', 'B-spline (guidance)');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'best');
title('Trajetoria');

subplot(3,2,2);
plot(guia.s, rad2deg(guia.alpha), 'b', 'LineWidth', 1);
grid on;
xlabel('s [m]'); ylabel('\alpha [deg]');
title('Heading da spline');

subplot(3,2,3);
plot(guia.s, guia.kappa, 'Color', [1 0.6 0.6], 'LineWidth', 0.8, ...
    'DisplayName', '\kappa B-spline'); hold on;
plot(guia.s, guia.kappa_smooth, 'r', 'LineWidth', 1.2, ...
    'DisplayName', sprintf('\\kappa smooth (fc=%.2f, ord=%d)', fc_kappa, ordem_kappa));
grid on;
xlabel('s [m]'); ylabel('\kappa [1/m]');
legend('Location', 'best');
title('Curvatura');

subplot(3,2,4);
plot(guia.s, guia.dkappa_ds, 'b', 'LineWidth', 1);
grid on;
xlabel('s [m]'); ylabel('d\kappa/ds [1/m^2]');
title('Derivada da curvatura');

subplot(3,2,[5 6]);
plot(1:n_orig, erros*100, 'k.-');
grid on;
xlabel('indice do wp original'); ylabel('erro [cm]');
title(sprintf('Distancia wp -> guia (max=%.1f cm, RMS=%.1f cm)', ...
    max(erros)*100, rms(erros)*100));
