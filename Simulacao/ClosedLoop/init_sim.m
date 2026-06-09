% init_sim.m — Inicializacao da simulacao closed-loop
%
% Carrega parametros do veiculo, gains do Controller, e trajetoria.
% Apos rodar este script, abrir sim_closedloop.slx
%
% Data: 2026-06-09 | Autor: Renan / Claude

clear; close all;

%% Parametros do veiculo
load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Planta', 'params', 'param_MF6713.mat'));

%% Condicoes iniciais da planta
% X0 = [x, y, psi, r, vy, omega_m, delta]
X0 = zeros(7, 1);
X0(2) = 1.0;  % erro lateral inicial de 1m

%% Velocidade longitudinal (m/s)
vx = 2.0;

%% Trajetoria
load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Guidance', 'trajetorias', 'reta_100m.mat'));
guidanceState = guidance_init(wps_reta);

%% Parametros do Controller (buses, gains, etc.)
run(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Controller', 'Param_Controller.m'));

%% Tempo de simulacao
Tsim = 50;  % s

fprintf('Closed-loop: vx=%.1f m/s | Tsim=%.0f s | traj=%s\n', vx, Tsim, 'reta_100m');
