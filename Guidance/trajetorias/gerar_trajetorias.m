% gerar_trajetorias.m — Gera trajetorias de teste (rodar uma vez)
%
% Data: 2026-06-09 | Autor: Renan / Claude

basedir = fileparts(mfilename('fullpath'));

% Reta de 100m na direcao x (waypoints a cada 3m)
wps_reta.x = (0:3:100)';
wps_reta.y = zeros(numel(wps_reta.x), 1);
save(fullfile(basedir, 'reta_100m.mat'), 'wps_reta');
fprintf('Salvo: reta_100m.mat (%d wps)\n', numel(wps_reta.x));

% Curva de raio 50m (arco de 180 graus, waypoints a cada 3m)
arc_len = pi * 50;  % ~157m
n_pts = round(arc_len / 3);
theta = linspace(0, pi, n_pts)';
wps_curva.x = 50 * sin(theta);
wps_curva.y = 50 * (1 - cos(theta));
save(fullfile(basedir, 'curva_R50.mat'), 'wps_curva');
fprintf('Salvo: curva_R50.mat (%d wps)\n', numel(wps_curva.x));
