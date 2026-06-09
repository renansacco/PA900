function Xp = dinamica_veiculo(X, U, p)
% DINAMICA_VEICULO  Bicycle-model vehicle dynamics (with optional implement).
%
%   Xp = dinamica_veiculo(X, U, p)
%
%   States (7x1):
%       X = [x; y; psi; r; vy; omega_m; delta]
%       x, y    - position in the global frame [m]
%       psi     - heading angle [rad]
%       r       - yaw rate [rad/s]
%       vy      - lateral velocity [m/s]
%       omega_m - steering motor speed [rad/s]
%       delta   - front-wheel steering angle [rad]
%
%   Inputs (2x1):
%       U = [omega_m_ref; vx]
%       omega_m_ref - steering motor speed reference [rad/s]
%       vx          - longitudinal velocity [m/s]
%
%   Params (struct p):
%       Cf   - front axle cornering stiffness [N/rad]
%       Cr   - rear axle cornering stiffness [N/rad]
%       Ch   - implement hitch cornering stiffness [N/rad] (0 = no implement)
%       Lf   - distance CG to front axle [m]
%       Lr   - distance CG to rear axle [m]
%       Lh   - distance CG to hitch point [m]
%       Lcp  - distance CG to control point [m]
%       Izz  - yaw moment of inertia [kg*m^2]
%       m    - vehicle mass [kg]
%       tau  - steering actuator time constant [s]
%       km   - steering motor gain [rad/(rad/s)]
%       kd   - steering gear ratio [-]
%
%   Output:
%       Xp - state derivative (7x1 column vector)
%
%   Date:   2026-06-09
%   Author: Renan / Claude

    % Unpack inputs
    omega_m_ref = U(1);
    vx          = U(2);

    % Unpack states
    x       = X(1);
    y       = X(2);
    psi     = X(3);
    r       = X(4);
    vy      = X(5);
    omega_m = X(6);
    delta   = X(7);

    % Unpack parameters
    Cf  = p.Cf;
    Cr  = p.Cr;
    Ch  = p.Ch;
    Lf  = p.Lf;
    Lr  = p.Lr;
    Lh  = p.Lh;
    Lcp = p.Lcp;
    Izz = p.Izz;
    m   = p.m;
    tau = p.tau;
    km  = p.km;
    kd  = p.kd;

    % --- Kinematics ---
    xp   = vx.*cos(psi) - (Lcp.*r + vy).*sin(psi);
    yp   = vx.*sin(psi) + (Lcp.*r + vy).*cos(psi);
    psip = r;

    % --- Front tyre slip angle ---
    alpha_f = atan((-vx.*sin(delta) + (Lf.*r + vy).*cos(delta)) ...
                ./ (vx.*cos(delta) + (Lf.*r + vy).*sin(delta)));

    % --- Rear tyre slip angle ---
    alpha_r = atan((-Lr.*r + vy) ./ vx);

    % --- Front and rear force contributions ---
    Fy_f = -Cf .* cos(delta) .* alpha_f;
    Fy_r = -Cr .* alpha_r;

    Mz_f = -Cf .* Lf .* cos(delta) .* alpha_f;
    Mz_r =  Cr .* Lr .* alpha_r;

    % --- Implement (hitch) contributions ---
    if Ch ~= 0
        alpha_h = atan((r.*(-Lh - Lr) + vy) ./ vx);
        Fy_h = -Ch .* alpha_h;
        Mz_h = -Ch .* (-Lh - Lr) .* alpha_h;
    else
        Fy_h = 0;
        Mz_h = 0;
    end

    % --- Lateral dynamics ---
    rp  = (Mz_f + Mz_r + Mz_h) ./ Izz;
    vyp = -r.*vx + (Fy_f + Fy_r + Fy_h) ./ m;

    % --- Steering actuator ---
    omega_m_p = (-1/tau) .* omega_m + (1/tau) .* omega_m_ref;
    delta_p   = km .* kd .* omega_m;

    % --- Output (column vector) ---
    Xp = [xp; yp; psip; rp; vyp; omega_m_p; delta_p];

end
