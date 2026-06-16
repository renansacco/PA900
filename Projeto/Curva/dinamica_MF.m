function Xp=dinamica_MF(t,X)
global Cf Cr Ch Lf Lr Lh Lcp Izz m tau km kd

U=controle(t,X);
Xp=dinamica(X,U,Cf,Cr,Ch,Lf,Lr,Lh,Lcp,Izz,m,tau,km,kd);