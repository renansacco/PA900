% gerar_trajetorias.m — Gera trajetorias para o benchmark do controlador de curva
%
% Cria 3 trajetorias (reta, curva fechada, taipa real) e salva em benchmark_trajs.mat
% Waypoints espacados a cada wpDist metros.

wpDist = 2.0;  % distancia entre waypoints [m]

%% 1. Reta sintetica (200m)
len_reta = 200;
n_reta = round(len_reta / wpDist) + 1;
trajs(1).name = 'reta';
trajs(1).wps.x = linspace(0, len_reta, n_reta)';
trajs(1).wps.y = zeros(n_reta, 1);

%% 2. Reta + curva fechada + reta (cabeceira sintetica)
R = 6;
len_reta_cabeceira = 30;  % metros de reta antes e depois da curva

% Trecho 1: reta em +x
n1 = round(len_reta_cabeceira / wpDist);
x1 = linspace(0, len_reta_cabeceira, n1 + 1)';
y1 = zeros(size(x1));

% Trecho 2: semicirculo R=6m, centro em (len_reta_cabeceira, R)
n2 = round(pi * R / wpDist);
theta = linspace(0, pi, n2 + 1)';
theta = theta(2:end);  % remove ponto duplicado com final da reta
x2 = len_reta_cabeceira + R * sin(theta);
y2 = R * (1 - cos(theta));

% Trecho 3: reta em -x (volta)
n3 = round(len_reta_cabeceira / wpDist);
x3 = linspace(len_reta_cabeceira, 0, n3 + 1)';
x3 = x3(2:end);  % remove ponto duplicado
y3 = 2 * R * ones(size(x3));

trajs(2).name = 'cabeceira';
trajs(2).wps.x = [x1; x2; x3];
trajs(2).wps.y = [y1; y2; y3];

%% 3. Taipa real (taipas_boeck, guia 20 — contem reta+curva+reta)
tmp = load('taipas_boeck.mat');
trajs(3).name = 'taipa_real';
trajs(3).wps = tmp.guias{20};

%% Salva
outFile = fullfile(fileparts(mfilename('fullpath')), 'benchmark_trajs.mat');
save(outFile, 'trajs');
fprintf('Salvo benchmark_trajs.mat com %d trajetorias\n', numel(trajs));

%% Preview
figure('Name', 'Benchmark — Trajetorias');
for k = 1:numel(trajs)
    subplot(1, 3, k);
    plot(trajs(k).wps.x, trajs(k).wps.y, '.-');
    axis equal; grid on;
    title(trajs(k).name, 'Interpreter', 'none');
    xlabel('x [m]'); ylabel('y [m]');
end
