% init_sim.m — Inicializacao da simulacao closed-loop
%
% Carrega parametros do veiculo, gains do Controller, e trajetoria.
% Apos rodar este script, abrir sim_closedloop.slx
%
% Data: 2026-06-09 | Autor: Renan / Claude

clear; close all;

%% Parametros do veiculo
params = load('param_MF6713.mat');

%% Guidance
Ts_guidance = 0.05;
useCourse=1;
tmp = load('taipas_boeck.mat');
%wps = [tmp.guias{10}.x, tmp.guias{10}.y];
wps = tmp.guias{20};

%% Condicoes iniciais da planta
% X0 = [x, y, psi, r, vy, omega_m, delta]
X0 = zeros(7, 1);
X0(1) = wps.x(1);
X0(2) = wps.y(1);
X0(3) = atan2(wps.y(2) - wps.y(1), wps.x(2) - wps.x(1));  % psi0 alinhado com a guia

%% Velocidade longitudinal (m/s)
vx = 2.0;

%% Parametros do Controller (buses, gains, etc.)
Param_Controller;

%% Tempo de simulacao
pathLen = sum(sqrt(diff(wps.x).^2 + diff(wps.y).^2));
Tsim = ceil(pathLen / vx) + 10;

fprintf('Closed-loop: vx=%.1f m/s | Tsim=%.0f s | pathLen=%.0f m\n', vx, Tsim, pathLen);
