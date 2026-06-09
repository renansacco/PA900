Gains = load('Gains\Implemento_Medio\Keep_Tractor_Implemento_Medio.mat');
% Gains = load('Gains\Sem_Implemento\Keep_Tractor_Sem_Implemento.mat');
% Gains = load('Gains\Implemento_Leve\Keep_Tractor_Implemento_Leve.mat');
Plant = load('param_MF6713_Sulcon5.mat');

vx = 3.0;
iV = find(Gains.vx_table==vx);

for iAg=1:7
    k=reshape(Gains.Gains_Keep_Tractor(iV, iAg, :), 1, 3);
    Analise_MF(Plant, vx, k);
end