% script_linearizar.m — Linearizacao numerica de dinamica_veiculo
%
% Calcula e exibe as matrizes A e B em torno do ponto de equilibrio.

clear; close all;

%% Parametros
paramFile = 'param_MF6713.mat';
p = load(fullfile('params', paramFile));

vx = 3.0;

%% Ponto de equilibrio
Xe = zeros(7, 1);
Ue = [0; vx];

%% Linearizacao
[A, B] = linearizar_veiculo(Xe, Ue, p);


%% Feedforward
% X = [x,y,psi,r,vy,omega_m,delta]
% X_ff = [- , -, alpha, k*vx, ?, ?, ?]
% Xp_ff = [-, -. r_ff, dkds*vx^2, ?, ?, ?]

% inputs
dkappa_ds = 0.1;
kappa = 0.2;

K_beta = 0.658;
K_delta = 2.486;

% X_ff = [- , -, alpha, k*vx, ?, ?, ?]
r_ff  = vx*kappa;
vy_ff = vx * K_beta * kappa;      % porque beta = vy/vx = K_beta*kappa  ->  vy = vx*K_beta*kappa
wm_ff = 0;
d_ff  = K_delta * kappa;

rp_ff  = vx     * vx * dkappa_ds;      % = vx^2 * dkappa_ds
vyp_ff = vx*K_beta * vx * dkappa_ds;   % = vx^2 * K_beta * dkappa_ds
omega_m_ff = (K_delta * vx * dkappa_ds) / p.k_d / p.k_m;
dp_ff  = K_delta * vx * dkappa_ds;

X_ff  = [vx*kappa; vx*K_beta*kappa; omega_m_ff; K_delta*kappa];
Xp_ff = [vx^2*dkappa_ds; vx^2*K_beta*dkappa_ds; omega_m_p_ff; K_delta*vx*dkappa_ds];

u_ff = pinv([0;0;0;0;0;5;0]) * (Xp_ff - A * X_ff)
%% Nomes dos estados e entradas
estados = {'x', 'y', 'psi', 'r', 'vy', 'omega_m', 'delta'};
entradas = {'omega_m_ref', 'vx'};
nx = numel(estados);
nu = numel(entradas);

%% Exibir estados
fprintf('=== Linearizacao de dinamica_veiculo ===\n');
fprintf('  Parametros: %s\n', paramFile);
fprintf('  vx = %.2f m/s\n\n', vx);

fprintf('Estados (7):\n');
for i = 1:nx
    fprintf('  X(%d) = %-10s  Xe = %g\n', i, estados{i}, Xe(i));
end
fprintf('\nEntradas (2):\n');
for i = 1:nu
    fprintf('  U(%d) = %-14s  Ue = %g\n', i, entradas{i}, Ue(i));
end

%% Exibir A
fprintf('\n--- Matriz A (7x7) ---\n');
fprintf('%12s', '');
for j = 1:nx
    fprintf('%12s', estados{j});
end
fprintf('\n');
for i = 1:nx
    fprintf('%12s', [estados{i} '''']);
    for j = 1:nx
        fprintf('%12.4f', A(i,j));
    end
    fprintf('\n');
end

%% Exibir B
fprintf('\n--- Matriz B (7x2) ---\n');
fprintf('%12s', '');
for j = 1:nu
    fprintf('%14s', entradas{j});
end
fprintf('\n');
for i = 1:nx
    fprintf('%12s', [estados{i} '''']);
    for j = 1:nu
        fprintf('%14.4f', B(i,j));
    end
    fprintf('\n');
end

%% Polos em malha aberta
polos = eig(A);
fprintf('\n--- Polos (malha aberta) ---\n');
for i = 1:nx
    if imag(polos(i)) == 0
        fprintf('  p%d = %.4f\n', i, real(polos(i)));
    elseif imag(polos(i)) > 0
        fprintf('  p%d = %.4f %+.4fj  (wn=%.4f, zeta=%.4f)\n', ...
            i, real(polos(i)), imag(polos(i)), ...
            abs(polos(i)), -real(polos(i))/abs(polos(i)));
    end
end

%% Controlabilidade e observabilidade (modelo reduzido 3:7)
Ar = A(3:7, 3:7);
Br = B(3:7, 1);
Cr = eye(5);

rank_C = rank(ctrb(Ar, Br));
rank_O = rank(obsv(Ar, Cr));

fprintf('\n--- Modelo reduzido (estados 3:7) ---\n');
fprintf('  Estados: psi, r, vy, omega_m, delta\n');
fprintf('  Rank de controlabilidade: %d / 5\n', rank_C);
fprintf('  Rank de observabilidade:  %d / 5\n', rank_O);
