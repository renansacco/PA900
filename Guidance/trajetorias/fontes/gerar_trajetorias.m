% gerar_trajetorias.m — Gera trajetorias de teste (rodar uma vez)
%
% Data: 2026-06-09 | Autor: Renan / Claude

outdir = fullfile(fileparts(mfilename('fullpath')), '..', 'waypoints');

% Reta de 100m na direcao x (waypoints a cada 3m)
wps_reta.x = (0:3:100)';
wps_reta.y = zeros(numel(wps_reta.x), 1);
save(fullfile(outdir, 'reta_100m.mat'), 'wps_reta');
fprintf('Salvo: reta_100m.mat (%d wps)\n', numel(wps_reta.x));

% Curva de raio 50m (arco de 180 graus, waypoints a cada 3m)
arc_len = pi * 50;  % ~157m
n_pts = round(arc_len / 3);
theta = linspace(0, pi, n_pts)';
wps_curva.x = 50 * sin(theta);
wps_curva.y = 50 * (1 - cos(theta));
save(fullfile(outdir, 'curva_R50.mat'), 'wps_curva');
fprintf('Salvo: curva_R50.mat (%d wps)\n', numel(wps_curva.x));

% Curvatura constante (kappa=0.05 1/m -> R=20m, arco de 200m)
ds = 3;
kappa_const = 0.05;
L_const = 200;
s = (0:ds:L_const)';
alpha = kappa_const * s;
wps_kconst.x = cumtrapz(s, cos(alpha));
wps_kconst.y = cumtrapz(s, sin(alpha));
save(fullfile(outdir, 'curva_k_constante.mat'), 'wps_kconst');
fprintf('Salvo: curva_k_constante.mat (%d wps, kappa=%.3f)\n', numel(s), kappa_const);

% Clotoide (kappa rampa 0 -> 0.10 1/m em 200m)
kappa_max = 0.20;
L_clot = 200;
s = (0:ds:L_clot)';
kappa_ramp = (kappa_max / L_clot) * s;
alpha = cumtrapz(s, kappa_ramp);
wps_clotoide.x = cumtrapz(s, cos(alpha));
wps_clotoide.y = cumtrapz(s, sin(alpha));
save(fullfile(outdir, 'clotoide.mat'), 'wps_clotoide');
fprintf('Salvo: clotoide.mat (%d wps, kappa 0->%.3f)\n', numel(s), kappa_max);

% Degrau em kappa (reta 50m + curva kappa=0.05 por 150m)
kappa_step = 0.05;
s1 = 50;
L_step = 200;
s = (0:ds:L_step)';
kappa_deg = zeros(size(s));
kappa_deg(s >= s1) = kappa_step;
alpha = cumtrapz(s, kappa_deg);
wps_degrau.x = cumtrapz(s, cos(alpha));
wps_degrau.y = cumtrapz(s, sin(alpha));
save(fullfile(outdir, 'degrau_kappa.mat'), 'wps_degrau');
fprintf('Salvo: degrau_kappa.mat (%d wps, kappa=0->%.3f em s=%.0fm)\n', numel(s), kappa_step, s1);

% Serpentina (kappa = A*sin(2*pi*s/lambda), A=0.05 1/m, lambda=100m, 400m)
A_serp      = 0.12;       % ou 0.15 p/ cobrir o pior ponto da taipa
lambda_serp = 12;
n_ciclos    = 4;
L_serp      = n_ciclos * lambda_serp;   % inteiro de ciclos -> termina em kappa=0
ds          = 0.05;                      % denso p/ cumtrapz e p/ spline fiel

s = (0:ds:L_serp)';
kappa_serp = A_serp * sin(2*pi*s / lambda_serp);
alpha = cumtrapz(s, kappa_serp);
alpha = alpha - mean(alpha);
x_dense = cumtrapz(s, cos(alpha));
y_dense = cumtrapz(s, sin(alpha));

% Reamostra waypoints a 3 m (igual a taipa) antes do guidance
wps_dense.x = x_dense;  wps_dense.y = y_dense;
wps = wps_dense;
save(fullfile(outdir, 'serpentina_dense.mat'), 'wps');