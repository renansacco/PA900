% init_sim.m — Inicializacao da simulacao de planta aberta
%
% Carrega parametros do veiculo e define condicoes iniciais.
% Apos rodar este script, abrir sim_planta.slx
%
% Data: 2026-06-09 | Autor: Renan / Claude

clear; close all;

%% Parametros do veiculo (alterar conforme necessario)
params = load('param_MF6713.mat');

%% Condicoes iniciais
% X0 = [x, y, psi, r, vy, omega_m, delta]
X0 = zeros(7, 1);

%% Velocidade longitudinal (m/s)
vx = 2.0;

%% Tempo de simulacao
Tsim = 30;  % s

%% Parametros disponiveis no workspace: parametros (struct), X0, vx, Tsim
fprintf('Planta aberta: %s | vx=%.1f m/s | Tsim=%.0f s\n', 'param_MF6713', vx, Tsim);
