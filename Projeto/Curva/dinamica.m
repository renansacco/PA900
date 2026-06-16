function [Xp] = dinamica(X,U,Cf,Cr,Ch,Lf,Lr,Lh,Lcp,Izz,m,tau,km,kd)

% Input
omegam_ref = U(1); vx = U(2);

% Estado atual
x = X(1); y = X(2); psi = X(3); r = X(4); vy = X(5); delta = X(6); omegam = X(7);

xp = vx.*cos(psi) - (Lcp.*r + vy).*sin(psi);
yp = vx.*sin(psi) + (Lcp.*r + vy).*cos(psi);

psip = r;
rp = (-Cf.*Lf.*cos(delta).*atan((-vx.*sin(delta) + (Lf.*r + vy).*cos(delta))./(vx.*cos(delta) + (Lf.*r + vy).*sin(delta))) - Ch.*(-Lh - Lr).*atan((r.*(-Lh - Lr) + vy)./vx) + Cr.*Lr.*atan((-Lr.*r + vy)./vx))./Izz;
vyp = -r.*vx + (-Cf.*cos(delta).*atan((-vx.*sin(delta) + (Lf.*r + vy).*cos(delta))./(vx.*cos(delta) + (Lf.*r + vy).*sin(delta))) - Ch.*atan((r.*(-Lh - Lr) + vy)./vx) - Cr.*atan((-Lr.*r + vy)./vx))./m;
omegamp = (-1/tau)*omegam + (1/tau)*omegam_ref;
deltap = km*kd*omegam;

Xp = [xp, yp, psip, rp, vyp, deltap, omegamp]';