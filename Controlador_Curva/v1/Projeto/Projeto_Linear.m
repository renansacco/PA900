%% Projeto do controlador linear
global Cf Cr Ch Lf Lr Lh Lcp Izz m tau km kd
global X0 psif K_psi K_r tsim ts

param = load('param_MF6713.mat');
Cf = param.Cf;
Cr = param.Cr;
Ch = param.Ch;
Lh = param.Lh;
Lf = param.Lf;
Lr = param.Lr;
Lcp = param.L_cp;
Izz = param.Izz;
m = param.m;
tau = param.tau;
km = param.k_m;
kd =  param.k_d;

% Condição inicial do veículo e do atuador
x0 = 0; y0 = 0; r0 = 0; vy0 = 0; psi0 = 0; delta0=0; omegam0=0;
X0 = [x0,y0,psi0,r0,vy0,delta0,omegam0]';

% Ganhos do controlador linearizado, para Y = [psie, r]
Ks = [41.28, 68.9];

K_psi=Ks(1); K_r=Ks(2); psif = deg2rad(10); tsim=10; ts=1;
opts=optimset('Display','iter','MaxIter',50,'TolX',1e-8);
Ks=fminsearch(@objetivo,Ks,opts)    

% % % H = [1,0,0,0,0;
% % %     0,0,1,0,0];
% % % % H = [1,0,0,0,0];
% % % 
% % % 
% % % J = eye(2);
% % % 
% % % 
% % % Ca = [-J*H];
% % % Fa = [J];
% % % 
% % % 
% % % %% Sistema em MF
% % % % der(x) = Ac*x + Bc*r
% % % Ac = A - B*k*Ca;
% % % Bc = -B*k*Fa;
% % % 
% % % sys_mf = ss(Ac, Bc, [1,0,0,0,0], 0)
% % % 
% % % figure
% % % bode(sys_mf)
% % % 
% % % figure
% % % step(sys_mf)
