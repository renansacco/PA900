%% Projeto do controlador linear para a planta Tractor - Lookup Table
% Gains_Keep_Tractor(iv, ir, ig)
% iv: indice da velocidade frontal  - vx_table
% ir: indice da agressividade, de 1 a 7 - R_table
% ig: indice do ganho, [k_psie, k_r, k_e]

global Cf Cr Ch Lf Lr Lh Lcp Izz m tau km kd
global X0 psif K_psi K_r K_e tsim ts 
global R Q P Pr vx

vx_table = 0.5:0.5:4.0;
R_table = 1./([0.3, 0.5, 0.8, 1.1, 1.4, 1.7, 2.0].^2);%flip(logspace(log10(1/10^2), log10(1/2^2), 7));

vx_table = 1.5;
R_table = 1/2^2;

%%
param = load('param_MF6713_Sulcon3.mat');
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
x0 = 0; y0 = 0; r0 = 0; vy0 = 0; psi0 = 0; delta0=0; omegam0=0; e0=0.1;
X0 = [x0,y0,psi0,r0,vy0,delta0,omegam0, e0]';

% Ganhos do controlador linearizado, para Y = [psie, r, e]
Ks = [39.9699   74.7483    9.9668];
K_psi=Ks(1); K_r=Ks(2); K_e = Ks(3); psif = deg2rad(10); tsim=30; ts=1;

vx=3;

% Pesos da função de custo
% [xp, yp, psip, rp, vyp, deltap, omegamp, ep]
R=0;%1/4^2;
Pr = 0;
Q = diag([0,0,32,0,0,0,0,50]);
P = diag([0,0,0,0,0,0,0,0]);

Gains_Keep_Tractor = zeros(size(vx_table,2), size(R_table,2), 3);
for iv = 1:size(vx_table,2)
    for ir = 1:size(R_table,2)
        
        vx = vx_table(iv);
        R = R_table(ir);
        opts=optimset('Display','off','MaxIter',100,'TolX',1e-6);
        Ks=fminsearch(@objetivo,Ks,opts)
        
        Gains_Keep_Tractor(iv, ir, :) = Ks;
        
        disp(iv*ir)
    end
end

save('Gains\Keep_Tractor_Sulcon', 'Gains_Keep_Tractor', 'vx_table', 'R_table')





