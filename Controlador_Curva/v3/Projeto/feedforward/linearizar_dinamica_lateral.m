%% ========================================================================
%  FF LINEAR - PASSO 1: LINEARIZAR E REDUZIR AOS ESTADOS [r, vy, delta]
%  ------------------------------------------------------------------------
%  Objetivo: obter A (3x3) e B (3x1) do subsistema que importa para o
%  feedforward de curva, com entrada = omega_m_ref.
%
%  Estado completo do dinamica_veiculo: [x, y, psi, r, vy, omega_m, delta]
%                                         1  2   3   4   5     6       7
%
%  Para o FF descartamos:
%    - x, y  (cinematica de posicao, nao afeta a dinamica)
%    - psi   (so integra r; nao entra em rp, vyp, deltap -> irrelevante p/ u_ff)
%
%  Restam os estados que definem o equilibrio de curva: [r, vy, delta]
%  Mas omega_m (estado 6) e o atuador: deltap = k*omega_m. Atuando "direto
%  em omega_m" (sem a dinamica do motor), tratamos omega_m como ENTRADA.
%  Entao:
%    - estados do subsistema: [r, vy, delta]  (indices 4,5,7)
%    - entrada: omega_m (que aparece via deltap = k*omega_m)
%
%  Como omega_m e o estado 6 e estamos tratando-o como entrada, extraimos
%  a coluna 6 da matriz A linearizada como o vetor B (efeito de omega_m
%  sobre [r,vy,delta]).
%
%  Autor: Renan / Claude
%% ========================================================================

clear; close all;

%% Paths
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));

vx = 2.0;     % velocidade de operacao [m/s]

%% Linearizar a planta completa (Xe=0, reta)
Xe = zeros(7,1);
Ue = [0; vx];                       % [omega_m_ref; vx]
[A_full, B_full] = linearizar_veiculo(Xe, Ue, p);

% A_full e 7x7, B_full e 7x2 (entradas omega_m_ref e vx)

%% Reduzir aos estados [r, vy, delta] tratando omega_m como entrada
%  Estados de interesse: r=4, vy=5, delta=7
%  Entrada: omega_m = estado 6 (tratado como entrada direta)

idx_st  = [4, 5, 7];     % r, vy, delta
idx_in  = 6;             % omega_m (como entrada)

% A do subsistema: como [r,vy,delta] se afetam mutuamente
A = A_full(idx_st, idx_st);          % 3x3

% B do subsistema: como omega_m (estado 6) afeta [r,vy,delta]
% Vem da coluna 6 de A_full (acoplamento de omega_m nos estados de interesse)
B = A_full(idx_st, idx_in);          % 3x1

%% Mostrar resultado
fprintf('=== Subsistema linearizado [r, vy, delta], entrada omega_m (vx=%.1f) ===\n\n', vx);

fprintf('A (3x3):\n');
fprintf('         r          vy         delta\n');
nomes = {'r    ', 'vy   ', 'delta'};
for i = 1:3
    fprintf('  %s', nomes{i});
    for j = 1:3
        fprintf('  %10.4f', A(i,j));
    end
    fprintf('\n');
end

fprintf('\nB (3x1)  [efeito de omega_m]:\n');
for i = 1:3
    fprintf('  %s  %10.4f\n', nomes{i}, B(i));
end

%% Verificacao de estrutura esperada
fprintf('\n=== Sanidade ===\n');
fprintf('  B(delta) deve ser ~k = k_m*k_d = %.4f  (deltap = k*omega_m)\n', p.k_m*p.k_d);
fprintf('  B(delta) obtido:                  %.4f\n', B(3));
fprintf('  B(r), B(vy) devem ser ~0 (omega_m so move delta diretamente)\n');
fprintf('  B(r)=%.4e, B(vy)=%.4e\n', B(1), B(2));

%% Salvar para a proxima etapa
save('ff_linear_AB.mat', 'A', 'B', 'vx', 'p');
fprintf('\nMatrizes salvas em ff_linear_AB.mat (A, B, vx).\n');
fprintf('Proximo passo: montar x_ff(kappa) e calcular u_ff = B^+(xp_ff - A x_ff).\n');