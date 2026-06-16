% plotar_cenario.m — Plot detalhado de um cenario do benchmark
%
% Uso:
%   load('benchmark_results.mat');
%   plotar_cenario(results(5));        % plota o cenario #5
%   plotar_cenario(results(5), 'CME'); % com label do controlador

function plotar_cenario(r, ctrl_label)
    if nargin < 2, ctrl_label = ''; end

    out = r.out;
    wps = r.wps;

    titulo = sprintf('%s | %s | vx=%.1f m/s', r.traj_name, r.ic_name, r.vx);
    if ~isempty(ctrl_label)
        titulo = sprintf('[%s] %s', ctrl_label, titulo);
    end

    %% Extrai sinais
    t           = out.e.time;
    e           = out.e.signals.values;
    psi_deg     = out.psi_deg.signals.values;
    psi_ref_deg = out.psi_ref_deg.signals.values;
    r_yaw       = out.r.signals.values;
    omegam      = out.omegam.signals.values;
    omegam_ref  = out.omegam_ref.signals.values;
    delta_deg   = out.delta_deg.signals.values;
    curvature   = out.curvature.signals.values;
    vx_sig      = out.vx.signals.values;
    xy          = out.r_IC.signals.values;

    psi_cont     = rad2deg(unwrap(deg2rad(psi_deg)));
    psi_ref_cont = rad2deg(unwrap(deg2rad(psi_ref_deg)));

    %% Figura 1 — Trajetoria
    figure('Name', sprintf('Trajetoria — %s', titulo));
    plot(wps.x, wps.y, 'k--', 'DisplayName', 'Referencia'); hold on;
    plot(xy(:,1), xy(:,2), 'b', 'LineWidth', 1.5, 'DisplayName', 'Veiculo');
    axis equal; grid on;
    xlabel('x [m]'); ylabel('y [m]');
    legend('Location', 'best');
    title(titulo, 'Interpreter', 'none');

    %% Figura 2 — Controle (4x2 subplots)
    figure('Name', sprintf('Controle — %s', titulo));
    sgtitle(titulo, 'Interpreter', 'none');

    subplot(4,2,1);
    plot(t, e); grid on;
    ylabel('e [m]');
    title('Erro lateral');

    subplot(4,2,2);
    plot(t, psi_cont, t, psi_ref_cont, '--'); grid on;
    ylabel('[deg]');
    legend('\psi', '\psi_{ref}', 'Location', 'best');
    title('Heading');

    subplot(4,2,3);
    plot(t, psi_cont - psi_ref_cont); grid on;
    ylabel('[deg]');
    title('Erro angular');

    subplot(4,2,4);
    plot(t, r_yaw); grid on;
    ylabel('r [rad/s]');
    title('Yaw rate');

    subplot(4,2,5);
    plot(t, omegam, t, omegam_ref, '--'); grid on;
    ylabel('[rad/s]');
    legend('\omega_m', '\omega_{m,ref}', 'Location', 'best');
    title('Velocidade angular motor');

    subplot(4,2,6);
    plot(t, delta_deg); grid on;
    ylabel('\delta [deg]');
    title('Angulo de estercamento');

    subplot(4,2,7);
    plot(out.curvature.time, curvature); grid on;
    ylabel('\kappa [1/m]');
    title('Curvatura');
    xlabel('Tempo [s]');

    subplot(4,2,8);
    plot(out.vx.time, vx_sig); grid on;
    ylabel('v_x [m/s]');
    title('Velocidade longitudinal');
    xlabel('Tempo [s]');

    %% Figura 3 — Metricas resumo
    if ~isempty(r.metrics)
        m = r.metrics;
        figure('Name', sprintf('Metricas — %s', titulo));
        sgtitle(titulo, 'Interpreter', 'none');

        subplot(1,2,1);
        edges = m.e_lat_hist_edges;
        centers = (edges(1:end-1) + edges(2:end)) / 2;
        bar(centers*100, m.e_lat_hist_counts, 1, 'FaceColor', [0.3 0.6 0.9]);
        grid on;
        xlabel('e_{lat} [cm]');
        ylabel('count');
        title(sprintf('Histograma (mean=%.1fcm, max=%.1fcm, rms=%.1fcm)', ...
            m.e_lat_mean*100, m.e_lat_max*100, m.e_lat_rms*100));

        subplot(1,2,2);
        labels = {'e_{mean}'; 'e_{max}'; 'e_{rms}'; 'energy'; 'smooth'; ...
                  'overshoot'; 't_{settle}'; 'sat_{wm}%'; 'sat_{\delta}%'};
        vals = [m.e_lat_mean*100; m.e_lat_max*100; m.e_lat_rms*100; ...
                m.ctrl_energy; m.ctrl_smooth; ...
                m.overshoot*100; m.settling_time; ...
                m.sat_omegam_pct; m.sat_delta_pct];
        barh(vals, 'FaceColor', [0.4 0.7 0.5]);
        set(gca, 'YTick', 1:numel(labels), 'YTickLabel', labels);
        grid on;
        xlabel('Valor');
        title('Metricas');
    end
end
