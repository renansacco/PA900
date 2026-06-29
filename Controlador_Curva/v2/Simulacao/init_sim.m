% init_sim.m — Simulacao closed-loop v2 (init + sim + plot)
%
% Carrega parametros, roda modelClosedLoop e plota resultados.
%
% Data: 2026-06-09 | Autor: Renan / Claude

clear; close all;

%% Paths
simDir     = fileparts(mfilename('fullpath'));
versionDir = fullfile(simDir, '..');
proj_root  = fullfile(versionDir, '..', '..');

addpath(fullfile(versionDir, 'ERT'));

%% Agressividade: 1=suave, 2=padrao, 3=agressivo
aggr_idx = 3;
aggr_names = {'suave', 'padrao', 'agressivo'};

pathName = 'taipas_boeck.mat';
%% Parametros do veiculo
params = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));

%% Guidance
Ts_guidance = 0.05;
useCourse = 1;
wpDistance = 1;

tmp = load(fullfile(proj_root, 'Guidance', 'trajetorias', 'guias', pathName));
guia = tmp.guia;

wps_original = struct('x', guia.x, 'y', guia.y);
wps = resample_waypoints(wps_original, wpDistance);

%% Condicoes iniciais da planta
X0 = zeros(7, 1);
X0(1) = wps.x(1);
X0(2) = wps.y(1);
X0(3) = atan2(wps.y(2) - wps.y(1), wps.x(2) - wps.x(1));

%% Velocidade longitudinal (m/s)
vx = 2;

%% Parametros do Controller (buses, gains, etc.)
Param_Controller;
userParameters.Value.curveAggressiveness = aggr_idx;

%% Tempo de simulacao
pathLen = sum(sqrt(diff(wps.x).^2 + diff(wps.y).^2));
Tsim = ceil(pathLen / vx);

fprintf('Closed-loop v2 [%s]: vx=%.1f m/s | Tsim=%.0f s | pathLen=%.0f m\n', ...
    aggr_names{aggr_idx}, vx, Tsim, pathLen);

%% Roda simulacao
modelName = 'modelClosedLoop_v2';
load_system(modelName);
out = sim(modelName);
fprintf('Simulacao concluida.\n');

%% Plot (usa plotar_cenario do Benchmark)
r.out          = out;
r.wps          = wps;
r.wps_original = wps_original;
r.wpDistance    = wpDistance;
r.traj_name    = pathName;
r.ic_name      = 'alinhado';
r.vx           = vx;
r.useCourse    = useCourse;
r.guia         = guia;
r.metrics      = [];
r.version      = 'v2';
r.aggr_name    = aggr_names{aggr_idx};
r.omegam_sat   = double(Controlador.Value.Curva.omegam_sat);
plotar_cenario(r);
