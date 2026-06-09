%% Projeto do controlador linear para a planta Tractor - Lookup Table
% Gains_Keep_Tractor(iv, ir, ig)
% iv: indice da velocidade frontal  - vx_table
% ir: indice da agressividade, de 1 a 7 - R_table
% ig: indice do ganho, [k_psie, k_r, k_e]

Desc = 'Projeto de regulador linear com realimentação das saídas [psi,r,e], baseado na minimização de função de custo quadrática, com e_ref. Produz um mapa de ganhos Gains_Keep_Tractor(iv, ir, ig).';
global Cf Cr Ch Lf Lr Lh Lcp Izz m tau km kd
global X0 psif K_psi K_r K_e K_omega tsim ts 
global R Q P Pr vx

vx_table = 0.5:0.5:4.0;
R_table = 1./([0.3, 0.5, 0.8, 1.1, 1.4, 1.7, 2.0].^2);%flip(logspace(log10(1/10^2), log10(1/2^2), 7));


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
Ks = [35.5321   59.3883    5.0975];
K_psi=Ks(1); K_r=Ks(2); K_e = Ks(3); psif = deg2rad(10); tsim=30; ts=1;


% Pesos da função de custo
% [xp, yp, psip, rp, vyp, deltap, omegamp, ep]

vx_table = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0];
R_table = [1/0.6^2, 1/0.8^2. 1/1.1^2, 1/1.4^2, 1/1.7^2, 1/2.0^2, 1/2.3^2];
tau_e_table = [6.00, 4.02, 2.68, 2.01, 1.61, 1.34, 1.15, 1.01];       % constante de tempo para erro lateral, para cada velocidade
tau_e_mult = [1.2, 1.15, 1.1, 1.0, 0.9, 0.85, 0.80];
% Ganhos para ITAE
% Pr = 0;
% Q = diag([0,0,0,0,0,0,0,4]);
% P = diag([0,0,15,15,0,0,0,0]);%50,80


% Ganhos QUADRATIC
Pr = 0;
Q = diag([0,0,0,0,0,0,0,50]);
P = diag([0,0,40,40,0,0,0,0]);%40,40




iplot = 0;

Gains_Keep_Tractor = zeros(size(vx_table,2), size(R_table,2), 3);
for iv = 1:size(vx_table,2)
    for ir = 1:size(R_table,2)
        
        vx = vx_table(iv);
        R = R_table(ir);
        opts=optimset('Display','off','MaxIter',50,'TolX',1e-4);
        Ks=fminsearch(@(k) objetivo_QUADRATIC(k,iplot, tau_e_table(iv)*tau_e_mult(ir)) ,Ks,opts)
        %Ks=fminsearch(@(k) objetivo_ITAE(k,iplot, tau_e_table(iv)) ,Ks,opts)
        
        %%
        fprintf('vx=%.2f, ir=%d\n', vx, ir)
        Analise_MF(param, vx, Ks)
        %%
        Gains_Keep_Tractor(iv, ir, :) = Ks;
    end
end

%% Salva ganhos obtidos
name = strcat('Gains\Implemento_Leve\Keep_Tractor_Implemento_Leve_',string(datetime('now','format','d-MMM-y-HH-mm-ss')));
save(name, 'Gains_Keep_Tractor', 'vx_table', 'R_table', 'Desc', 'Q', 'P')

%%
for i=1:7
    Analise_MF(param, vx, reshape(Gains_Keep_Tractor(1, i,:),1,3));
end



