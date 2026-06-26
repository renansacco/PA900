%% ========================================================================
%  FEEDFORWARD DINAMICO PARA RASTREIO DE CURVA COM CURVATURA VARIAVEL
%  ------------------------------------------------------------------------
%  CONCEITO
%
%  Queremos o sinal de feedforward u_ff que faz o veiculo seguir uma
%  trajetoria de curvatura kappa(s) -- inclusive quando kappa VARIA ao
%  longo do caminho (rampa, serpentina), nao so kappa constante.
%
%  Inversao dinamica da planta:
%       u_ff = B^+ * ( Xp_ff - A * X_ff )
%
%  - X_ff : estado de equilibrio que segue a trajetoria (depende de kappa)
%  - Xp_ff: derivada temporal de X_ff
%
%  Curvatura variavel -> X_ff = X_ff(kappa), e pela regra da cadeia:
%       Xp_ff = (dX_ff/dkappa) * kappa_dot,   kappa_dot = vx * dkappa_ds
%
%  Precisamos de:
%    (1) X_ff(kappa)   -> resolvendo o EQUILIBRIO para cada kappa (fsolve)
%    (2) dX_ff/dkappa  -> derivando X_ff(kappa) numericamente
%    (3) dkappa_ds     -> da trajetoria (tabela de curvatura)
%
%  Estado da planta (dinamica_veiculo.m):
%       X = [x, y, psi, r, vy, omega_m, delta]
%  Para u_ff usamos os estados dinamicos [r, vy, omega_m, delta].
%
%  Em kappa constante: Xp_ff=0 -> u_ff ~ 0 (consistente com FF estatico).
%  Em kappa variavel : Xp_ff!=0 -> u_ff != 0 = antecipacao de dkappa_ds.
%
%  Autor: Renan / Claude
%% ========================================================================

clear; close all;

%% ---- Carrega planta -----------------------------------------------------
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));

vx = 2.0;     % velocidade de operacao [m/s]

%% ========================================================================
%  PARTE 1 - X_ff(kappa) E dX_ff/dkappa
%  ------------------------------------------------------------------------
%  Resolve o equilibrio (equilibrio_kappa -> fsolve -> deriv_internas) para
%  uma faixa de kappa. Monta X_ff(kappa) = [r, vy, omega_m, delta] e deriva
%  numericamente em kappa.
%% ========================================================================

kappa_grid = linspace(-0.2, 0.2, 41);        % faixa de curvatura [1/m]
Xff_grid   = zeros(4, numel(kappa_grid));     % [r; vy; omega_m; delta] x kappa

for i = 1:numel(kappa_grid)
    Xff_grid(:,i) = equilibrio_kappa(kappa_grid(i), vx, p);
end

% Derivada dX_ff/dkappa (diferencas finitas ao longo da grade)
dXff_dkappa = gradient(Xff_grid, kappa_grid(2)-kappa_grid(1));

% Coeficientes de proporcionalidade com kappa (regiao linear) -- inspecao
i0 = find(kappa_grid > 0, 1, 'first');
K_r_coef     = Xff_grid(1,i0) / kappa_grid(i0);        % ~vx
K_beta_coef  = (Xff_grid(2,i0)/vx) / kappa_grid(i0);   % K_beta = (vy/vx)/kappa
K_delta_coef = Xff_grid(4,i0) / kappa_grid(i0);        % K_delta = delta/kappa

fprintf('=== Coeficientes de equilibrio (vx=%.1f) ===\n', vx);
fprintf('  r/kappa  = %.4f  (esperado vx=%.2f)\n', K_r_coef, vx);
fprintf('  K_beta   = %.5f\n', K_beta_coef);
fprintf('  K_delta  = %.5f  (cinematico L=%.3f)\n', K_delta_coef, p.Lf+p.Lr);

%% ========================================================================
%  PARTE 2 - PLANTA LINEARIZADA (A, B)
%  ------------------------------------------------------------------------
%  Linearizada em Xe=0 (reta), vx fixo. Estados dinamicos [r,vy,omega_m,delta]
%  = indices [4,5,6,7] do estado completo. Entrada = omega_m_ref.
%% ========================================================================

Xe = zeros(7,1);
Ue = [0; vx];
[A_full, B_full] = linearizar_veiculo(Xe, Ue, p);

idx = [4, 5, 6, 7];        % r, vy, omega_m, delta
A = A_full(idx, idx);      % 4x4
B = B_full(idx, 1);        % 4x1

%% ========================================================================
%  PARTE 3 - VALIDACAO
%  ------------------------------------------------------------------------
%  (a) kappa constante  -> u_ff ~ 0
%  (b) kappa variavel   -> u_ff != 0, linear em dkappa_ds
%% ========================================================================

fprintf('\n=== Validacao ===\n');

kappa_test = 0.1;
uff_const = feedforward_din(kappa_test, 0.0, vx, kappa_grid, Xff_grid, dXff_dkappa, A, B);
fprintf('  kappa const (0.1, dk/ds=0): u_ff = %.4e  (esperado ~0)\n', uff_const);

uff_din = feedforward_din(kappa_test, 0.02, vx, kappa_grid, Xff_grid, dXff_dkappa, A, B);
fprintf('  kappa=0.1, dk/ds=0.02:      u_ff = %.4f rad/s  (!= 0)\n', uff_din);

dk_vec  = linspace(-0.05, 0.05, 21);
uff_vec = zeros(size(dk_vec));
for i = 1:numel(dk_vec)
    uff_vec(i) = feedforward_din(kappa_test, dk_vec(i), vx, kappa_grid, Xff_grid, dXff_dkappa, A, B);
end

figure('Name','Feedforward dinamico');
subplot(1,2,1);
plot(kappa_grid, Xff_grid(4,:)*180/pi, 'b', 'LineWidth', 1.5); grid on;
xlabel('\kappa [1/m]'); ylabel('\delta_{ff} [deg]');
title('Esterco de equilibrio \delta_{ff}(\kappa)');

subplot(1,2,2);
plot(dk_vec, uff_vec, 'b', 'LineWidth', 1.5); grid on;
xlabel('d\kappa/ds [1/m^2]'); ylabel('u_{ff} = \omega_{m,ff} [rad/s]');
title(sprintf('FF dinamico vs d\\kappa/ds  (\\kappa=%.2f)', kappa_test));

sgtitle('Feedforward dinamico: equilibrio + antecipacao de d\kappa/ds');

%% ========================================================================
%  USO NA LEI DE CONTROLE
%  ------------------------------------------------------------------------
%  Para cada ponto (kappa, dkappa_ds da tabela do path):
%
%     uff      = feedforward_din(kappa, dkappa_ds, vx, kappa_grid, ...
%                                Xff_grid, dXff_dkappa, A, B);
%     delta_ff = interp1(kappa_grid, Xff_grid(4,:), kappa, 'pchip');
%
%     omega_m_ref = uff ...                          % FF dinamico (entrada)
%                 + K_psi  *(courseRef - psi) ...
%                 + K_r    *(vx*kappa   - r) ...
%                 + K_delta*(delta_ff   - delta);
%
%  Pre-compute kappa_grid/Xff_grid/dXff_dkappa UMA VEZ por vx (offline).
%  Em runtime: interp1 + multiplicacao 4x4.
%% ========================================================================


%% ========================================================================
%  FUNCOES LOCAIS  (ao final do arquivo, exigencia do MATLAB)
%% ========================================================================

function res = deriv_internas(z, vx, kappa, p)
% Residuo do equilibrio de curva para um kappa dado.
% z = [vy; delta; omega_m_ref]. Zera quando o veiculo esta em regime na
% curva: rp=0 (yaw estacionario), vyp=0 (lateral estacionario),
% omega_m_p=0 (atuador em regime; delta parado -> omega_m=0).
% O kappa entra como forcante via r = vx*kappa (estado imposto), que aparece
% dentro das equacoes de rp e vyp -- por isso o sistema NAO e homogeneo.
    vy          = z(1);
    delta       = z(2);
    omega_m_ref = z(3);

    r_target = vx * kappa;                 % taxa de yaw imposta pela curva

    X = [0; 0; 0; r_target; vy; 0; delta]; % x,y,psi irrelevantes; omega_m=0
    U = [omega_m_ref; vx];

    Xp = dinamica_veiculo(X, U, p);

    res = [ Xp(4);     % rp        = 0
            Xp(5);     % vyp       = 0
            Xp(6) ];   % omega_m_p = 0
end

function Xff = equilibrio_kappa(kappa, vx, p)
% Estado de equilibrio reduzido [r; vy; omega_m; delta] para um kappa.
% r = vx*kappa (imposto), omega_m = 0 (regime), vy e delta do fsolve.
    if abs(kappa) < 1e-9
        Xff = [0; 0; 0; 0];                % reta
        return;
    end
    z0   = [0; (p.Lf+p.Lr)*kappa; 0];      % chute: delta ~ L*kappa
    opts = optimoptions('fsolve','Display','off','TolFun',1e-12);
    z_eq = fsolve(@(z) deriv_internas(z, vx, kappa, p), z0, opts);

    vy_eq    = z_eq(1);
    delta_eq = z_eq(2);
    Xff = [vx*kappa; vy_eq; 0; delta_eq];  % [r, vy, omega_m, delta]
end

function uff = feedforward_din(kappa, dkappa_ds, vx, kappa_grid, Xff_grid, dXff_dkappa, A, B)
% Feedforward dinamico u_ff = B^+ (Xp_ff - A X_ff).
% X_ff e dX_ff/dkappa interpolados da grade; Xp_ff pela regra da cadeia.
    Xff  = interp1(kappa_grid, Xff_grid',    kappa, 'pchip')';   % [r,vy,omega_m,delta]
    dXff = interp1(kappa_grid, dXff_dkappa', kappa, 'pchip')';   % dX_ff/dkappa

    kappa_dot = vx * dkappa_ds;            % d(kappa)/dt = vx * d(kappa)/ds
    Xp_ff = dXff * kappa_dot;              % derivada temporal de X_ff

    uff = pinv(B) * (Xp_ff - A * Xff);     % inversao dinamica
end