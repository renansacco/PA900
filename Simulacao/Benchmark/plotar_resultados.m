% plotar_resultados.m — Gera graficos do benchmark do controlador de curva
%
% Carrega benchmark_results.mat e gera:
%   1. Tabela resumo no console
%   2. Histogramas de erro lateral por trajetoria
%   3. Barras comparativas das metricas principais
%   4. Heatmaps (trajetoria x velocidade) para IC alinhado

close all;

if ~exist('results', 'var')
    tmp = load('benchmark_results.mat');
    results = tmp.results;
end

%% Filtra resultados validos
valid = arrayfun(@(r) ~isempty(r.metrics), results);
res = results(valid);

traj_names = unique({res.traj_name}, 'stable');
ic_names   = unique({res.ic_name}, 'stable');
vels       = unique([res.vx]);

%% 1. Histogramas de erro lateral — uma figura por trajetoria, subplots por (IC x vx)
for it = 1:numel(traj_names)
    figure('Name', sprintf('Histograma — %s', traj_names{it}));
    sgtitle(sprintf('Erro lateral: %s', traj_names{it}), 'Interpreter', 'none');

    sp = 0;
    for ic = 1:numel(ic_names)
        for iv = 1:numel(vels)
            sp = sp + 1;
            subplot(numel(ic_names), numel(vels), sp);

            idx = strcmp({res.traj_name}, traj_names{it}) & ...
                  strcmp({res.ic_name}, ic_names{ic}) & ...
                  [res.vx] == vels(iv);
            if ~any(idx), continue; end
            r = res(idx);
            m = r.metrics;

            edges = m.e_lat_hist_edges;
            centers = (edges(1:end-1) + edges(2:end)) / 2;
            bar(centers*100, m.e_lat_hist_counts, 1, 'FaceColor', [0.3 0.6 0.9]);
            xlim([-50 50]);
            grid on;
            xlabel('e_{lat} [cm]');
            ylabel('count');
            title(sprintf('%s | vx=%.1f', ic_names{ic}, vels(iv)), 'Interpreter', 'none');
        end
    end
end

%% 2. Barras comparativas — e_lat_rms por (trajetoria, vx), agrupado por IC
figure('Name', 'Comparativo — RMS erro lateral');
sgtitle('RMS do erro lateral [m]');
for ic = 1:numel(ic_names)
    subplot(1, numel(ic_names), ic);

    data = zeros(numel(traj_names), numel(vels));
    for it = 1:numel(traj_names)
        for iv = 1:numel(vels)
            idx = strcmp({res.traj_name}, traj_names{it}) & ...
                  strcmp({res.ic_name}, ic_names{ic}) & ...
                  [res.vx] == vels(iv);
            if any(idx)
                data(it, iv) = res(idx).metrics.e_lat_rms;
            end
        end
    end

    bar(data);
    set(gca, 'XTickLabel', traj_names);
    legend(arrayfun(@(v) sprintf('vx=%.1f', v), vels, 'UniformOutput', false), ...
        'Location', 'best');
    ylabel('e_{rms} [m]');
    title(ic_names{ic}, 'Interpreter', 'none');
    grid on;
end

%% 3. Barras comparativas — controle (energia e suavidade) para IC alinhado
figure('Name', 'Comparativo — Sinal de controle');
idx_alin = strcmp({res.ic_name}, 'alinhado');

subplot(1,2,1);
data_energy = zeros(numel(traj_names), numel(vels));
for it = 1:numel(traj_names)
    for iv = 1:numel(vels)
        idx = idx_alin & strcmp({res.traj_name}, traj_names{it}) & [res.vx] == vels(iv);
        if any(idx), data_energy(it, iv) = res(idx).metrics.ctrl_energy; end
    end
end
bar(data_energy);
set(gca, 'XTickLabel', traj_names);
legend(arrayfun(@(v) sprintf('vx=%.1f', v), vels, 'UniformOutput', false), 'Location', 'best');
ylabel('RMS \omega_{m,ref} [rad/s]');
title('Energia de controle');
grid on;

subplot(1,2,2);
data_smooth = zeros(numel(traj_names), numel(vels));
for it = 1:numel(traj_names)
    for iv = 1:numel(vels)
        idx = idx_alin & strcmp({res.traj_name}, traj_names{it}) & [res.vx] == vels(iv);
        if any(idx), data_smooth(it, iv) = res(idx).metrics.ctrl_smooth; end
    end
end
bar(data_smooth);
set(gca, 'XTickLabel', traj_names);
legend(arrayfun(@(v) sprintf('vx=%.1f', v), vels, 'UniformOutput', false), 'Location', 'best');
ylabel('RMS d/dt(\omega_{m,ref}) [rad/s^2]');
title('Suavidade de controle');
grid on;

%% 4. Barras — saturacao para IC alinhado
figure('Name', 'Comparativo — Saturacao');
sgtitle('Saturacao (IC alinhado)');

subplot(1,2,1);
data_sat_wm = zeros(numel(traj_names), numel(vels));
for it = 1:numel(traj_names)
    for iv = 1:numel(vels)
        idx = idx_alin & strcmp({res.traj_name}, traj_names{it}) & [res.vx] == vels(iv);
        if any(idx), data_sat_wm(it, iv) = res(idx).metrics.sat_omegam_pct; end
    end
end
bar(data_sat_wm);
set(gca, 'XTickLabel', traj_names);
legend(arrayfun(@(v) sprintf('vx=%.1f', v), vels, 'UniformOutput', false), 'Location', 'best');
ylabel('% tempo');
title('|\omega_m| \geq 10 rad/s');
grid on;

subplot(1,2,2);
data_sat_d = zeros(numel(traj_names), numel(vels));
for it = 1:numel(traj_names)
    for iv = 1:numel(vels)
        idx = idx_alin & strcmp({res.traj_name}, traj_names{it}) & [res.vx] == vels(iv);
        if any(idx), data_sat_d(it, iv) = res(idx).metrics.sat_delta_pct; end
    end
end
bar(data_sat_d);
set(gca, 'XTickLabel', traj_names);
legend(arrayfun(@(v) sprintf('vx=%.1f', v), vels, 'UniformOutput', false), 'Location', 'best');
ylabel('% tempo');
title('|\delta| \geq 50 deg');
grid on;

%% 5. Resumo transiente — settling time e overshoot para ICs com erro
figure('Name', 'Comparativo — Transiente');
sgtitle('Transiente');

ic_erro = ~strcmp({res.ic_name}, 'alinhado');
res_trans = res(ic_erro);
labels = arrayfun(@(r) sprintf('%s\n%s\nvx=%.1f', r.traj_name, r.ic_name, r.vx), ...
    res_trans, 'UniformOutput', false);

subplot(2,1,1);
bar([res_trans.metrics]);  % won't work directly, extract manually:
overshoot_vals = arrayfun(@(r) r.metrics.overshoot, res_trans);
bar(overshoot_vals, 'FaceColor', [0.9 0.4 0.3]);
set(gca, 'XTick', 1:numel(labels), 'XTickLabel', labels, 'XTickLabelRotation', 45);
ylabel('Overshoot |e_{lat}| max [m]');
title('Overshoot (primeiros 5s)');
grid on;

subplot(2,1,2);
settle_vals = arrayfun(@(r) r.metrics.settling_time, res_trans);
bar(settle_vals, 'FaceColor', [0.4 0.8 0.5]);
set(gca, 'XTick', 1:numel(labels), 'XTickLabel', labels, 'XTickLabelRotation', 45);
ylabel('Settling time [s]');
title('Tempo de acomodacao (|e_{lat}| < 10cm)');
grid on;
