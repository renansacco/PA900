function Xp = dinamica_veiculo_cinematico(X, U, p)
% DINAMICA_VEICULO_CINEMATICO  Kinematic bicycle model (no tire dynamics).
%
%   Replica exata do modelo usado em vehicle_simulator.cpp (embarcado).
%   Drop-in replacement para dinamica_veiculo.m — mesmos estados/entradas.
%
%   Diferencas vs modelo dinamico:
%     - SEM forcas laterais de pneu (Cf, Cr, Ch ignorados)
%     - SEM inercia (m, Izz ignorados)
%     - SEM dinamica do atuador (resposta instantanea, tau ignorado)
%     - r = yaw rate geometrico (nao dinamico)
%     - vy = Lr*r (cinematico, sem slip de pneu)
%
%   States (7x1):  [x, y, psi, r, vy, omega_m, delta]
%   Inputs  (2x1): [omega_m_ref, vx]
%   Params: usa apenas Lf, Lr, L_cp, k_m, k_d

    omega_m_ref = U(1);
    vx          = U(2);

    psi     = X(3);
    r       = X(4);
    vy      = X(5);
    omega_m = X(6);
    delta   = X(7);

    L   = p.Lf + p.Lr;
    Lcp = p.L_cp;

    % Sideslip geometrico (exato como vehicle_simulator.cpp)
    beta = atan(p.Lr * tan(delta) / L);

    % Yaw rate cinematico
    psip = vx * cos(beta) * tan(delta) / L;

    % Velocidade lateral no CG (cinematica: vy_CG = vx*tan(beta) = Lr*r)
    vy_cg = vx * tan(beta);

    % Posicao do ponto de controle (mesma formula do dinamico)
    xp = vx * cos(psi) - (vy_cg + Lcp * psip) * sin(psi);
    yp = vx * sin(psi) + (vy_cg + Lcp * psip) * cos(psi);

    % r e vy: tracking rapido para manter interface compativel
    tau_fast = 0.005;
    rp  = (psip - r)   / tau_fast;
    vyp = (vy_cg - vy) / tau_fast;

    % Atuador: resposta instantanea (sem tau)
    omega_m_p = (omega_m_ref - omega_m) / tau_fast;
    delta_p   = p.k_m * p.k_d * omega_m;

    Xp = [xp; yp; psip; rp; vyp; omega_m_p; delta_p];

end
