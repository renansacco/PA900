function Xp = dinamica_veiculo(X, U, p)
% Modelo bicicleta nao-linear, 7 estados.
% Recebe parametros como struct (campos de param_MF6713.mat).

omegam_ref = U(1); vx = U(2);

psi = X(3); r = X(4); vy = X(5); delta = X(6); omegam = X(7);

% Angulos de derrapagem dos pneus
alpha_f = atan((-vx*sin(delta) + (p.Lf*r + vy)*cos(delta)) / ...
               (vx*cos(delta) + (p.Lf*r + vy)*sin(delta)));
alpha_r = atan((-p.Lr*r + vy) / vx);
alpha_h = atan((r*(-p.Lh - p.Lr) + vy) / vx);

% Cinematica
xp = vx*cos(psi) - (p.L_cp*r + vy)*sin(psi);
yp = vx*sin(psi) + (p.L_cp*r + vy)*cos(psi);
psip = r;

% Dinamica lateral
rp  = (-p.Cf*p.Lf*cos(delta)*alpha_f - p.Ch*(-p.Lh - p.Lr)*alpha_h + p.Cr*p.Lr*alpha_r) / p.Izz;
vyp = -r*vx + (-p.Cf*cos(delta)*alpha_f - p.Ch*alpha_h - p.Cr*alpha_r) / p.m;

% Atuador
omegamp = (-omegam + omegam_ref) / p.tau;
deltap  = p.k_m * p.k_d * omegam;

Xp = [xp; yp; psip; rp; vyp; deltap; omegamp];
end
