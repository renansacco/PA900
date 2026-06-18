% rodar_benchmark.m — Roda o benchmark do controlador de curva
%
% Itera sobre combinacoes de trajetoria, condicao inicial e velocidade.
% Usa modelClosedLoop.slx com os ganhos carregados por Param_Controller.
%
% Pre-requisito: rodar gerar_trajetorias.m antes (gera benchmark_trajs.mat)

clear; close all;

%% Paths
benchDir = fileparts(mfilename('fullpath'));
addpath(fullfile(benchDir, 'trajetorias'));

%% Config
velocidades = [1.0, 2.0, 3.0];  % m/s
t_descarte  = 2.0;              % s — descarte para metricas de regime
useCourse   = 1;                % 0 = heading, 1 = course (psi+beta) no lookahead
wpDistance_guidance = 3.0;       % resample waypoints (igual ao embarcado). 0 = sem resample

condicoes_iniciais = {
    'alinhado',     0,    0;      % nome, offset_lateral [m], erro_heading [deg]
    'offset_05m',   0.5,  0;
    'heading_10deg', 0,   10;
};

%% Carrega trajetorias
tmp = load(fullfile(benchDir, 'trajetorias', 'benchmark_trajs.mat'));
trajs = tmp.trajs;

%% Carrega parametros do veiculo e controller
params = load('param_MF6713.mat');
Ts_guidance = 0.05;
Param_Controller;

%% Carrega modelo uma vez
modelName = 'modelClosedLoop';
load_system(modelName);

%% Loop principal
n_trajs = numel(trajs);
n_ics   = size(condicoes_iniciais, 1);
n_vels  = numel(velocidades);
n_total = n_trajs * n_ics * n_vels;

results = struct([]);
idx = 0;

fprintf('=== Benchmark: %d simulacoes ===\n', n_total);

for it = 1:n_trajs
    for ic = 1:n_ics
        for iv = 1:n_vels
            idx = idx + 1;
            vx = velocidades(iv);

            ic_name    = condicoes_iniciais{ic, 1};
            ic_offset  = condicoes_iniciais{ic, 2};
            ic_heading = condicoes_iniciais{ic, 3};

            %% Trajetoria (resample como o embarcado)
            wps_original = trajs(it).wps;
            if wpDistance_guidance > 0
                wps = resample_waypoints(wps_original, wpDistance_guidance);
            else
                wps = wps_original;
            end

            %% Condicao inicial (alpha0 via spline do guidance, no 2o wp)
            state0 = guidance_init(wps);
            [g0, ~] = guidance_step([wps.x(2), wps.y(2), 0], state0);
            alpha0 = g0.alpha;

            X0 = zeros(7, 1);
            X0(1) = wps.x(2) - ic_offset * sin(alpha0);  % offset perpendicular
            X0(2) = wps.y(2) + ic_offset * cos(alpha0);
            X0(3) = alpha0 + deg2rad(ic_heading);
            % delta0 = 0 (controlador converge rapidamente para o valor correto)

            %% Tempo de simulacao
            pathLen = sum(sqrt(diff(wps.x).^2 + diff(wps.y).^2));
            Tsim = ceil(pathLen / vx) + 10;

            %% Roda simulacao
            scenarioStr = sprintf('[%d/%d] %s | %s | vx=%.1f', ...
                idx, n_total, trajs(it).name, ic_name, vx);
            fprintf('%s ... ', scenarioStr);

            try
                out = sim(modelName);
                metrics = calcular_metricas(out, t_descarte);
                fprintf('OK (e_rms=%.3fm, e_max=%.3fm)\n', ...
                    metrics.e_lat_rms, metrics.e_lat_max);
            catch ME
                fprintf('ERRO: %s\n', ME.message);
                metrics = [];
            end

            %% Armazena resultado
            r_entry.traj_name  = trajs(it).name;
            r_entry.ic_name    = ic_name;
            r_entry.ic_offset  = ic_offset;
            r_entry.ic_heading = ic_heading;
            r_entry.vx         = vx;
            r_entry.useCourse  = useCourse;
            r_entry.wpDistance = wpDistance_guidance;
            r_entry.wps        = wps;
            r_entry.wps_original = wps_original;
            r_entry.metrics    = metrics;
            r_entry.out        = out;

            if isempty(results)
                results = r_entry;
            else
                results(idx) = r_entry;
            end
        end
    end
end

%% Salva resultados
outFile = fullfile(fileparts(mfilename('fullpath')), 'benchmark_results_course.mat');
save(outFile, 'results');
fprintf('\nSalvo: benchmark_results.mat (%d cenarios)\n', numel(results));

%% Imprime tabela resumo
imprimir_tabela_resumo(results);

function imprimir_tabela_resumo(results)
    fprintf('\n%3s %-20s %-14s %5s | %7s %7s %7s | %7s %7s | %7s %6s | %6s %6s\n', ...
        'ID', 'Trajetoria', 'IC', 'vx', 'e_mean', 'e_max', 'e_rms', ...
        'energy', 'smooth', 'ovrsht', 't_set', 'sat_wm', 'sat_d');
    fprintf('%s\n', repmat('-', 1, 124));
    for i = 1:numel(results)
        r = results(i);
        if isempty(r.metrics), continue; end
        m = r.metrics;
        fprintf('%3d %-20s %-14s %5.1f | %7.3f %7.3f %7.3f | %7.2f %7.1f | %7.3f %5.1fs | %5.1f%% %5.1f%%\n', ...
            i, r.traj_name, r.ic_name, r.vx, ...
            m.e_lat_mean, m.e_lat_max, m.e_lat_rms, ...
            m.ctrl_energy, m.ctrl_smooth, ...
            m.overshoot, m.settling_time, ...
            m.sat_omegam_pct, m.sat_delta_pct);
    end
end
