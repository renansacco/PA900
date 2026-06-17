% plotar_cenario.m — Plot detalhado de um cenario do benchmark
%
% Uso:
%   load('benchmark_results.mat');
%   plotar_cenario(results(5));        % plota o cenario #5
%   plotar_cenario(results(5), 'CME'); % com label do controlador

function plotar_cenario(r, ctrl_label)
    if nargin < 2, ctrl_label = ''; end

    if isnumeric(r)
        tmp = load('benchmark_results.mat');
        r = tmp.results(r);
    end

    out = r.out;
    wps = r.wps;

    titulo = sprintf('%s | %s | vx=%.1f m/s', r.traj_name, r.ic_name, r.vx);
    if isfield(r, 'useCourse')
        if r.useCourse
            titulo = sprintf('%s | course', titulo);
        else
            titulo = sprintf('%s | heading', titulo);
        end
    end
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

    % Sideslip e course
    has_beta = false;
    try
        beta = out.beta.signals.values;
        has_beta = true;
    catch
    end

    % Alpha do guidance
    has_alpha = false;
    try
        alpha_guid = out.alpha.signals.values;
        has_alpha = true;
    catch
    end

    psi_rad = deg2rad(psi_deg);
    psi_ref_rad = deg2rad(psi_ref_deg);

    % Heading e course continuos (unwrap)
    psi_cont     = rad2deg(unwrap(psi_rad));
    psi_ref_cont = rad2deg(unwrap(psi_ref_rad));
    if has_beta
        course_cont = rad2deg(unwrap(psi_rad + beta));
    end
    if has_alpha
        alpha_cont = rad2deg(unwrap(alpha_guid));
    end

    %% Figura 1 — Trajetoria
    figure('Name', sprintf('Trajetoria — %s', titulo));
    plot(wps.x, wps.y, 'k--', 'DisplayName', 'Referencia'); hold on;
    plot(xy(:,1), xy(:,2), 'b', 'LineWidth', 1.5, 'DisplayName', 'Veiculo');
    axis equal; grid on;
    xlabel('x [m]'); ylabel('y [m]');
    legend('Location', 'best');
    title(titulo, 'Interpreter', 'none');

    %% Figura 2 — Controle (5x2 subplots)
    figure('Name', sprintf('Controle — %s', titulo));
    sgtitle(titulo, 'Interpreter', 'none');

    subplot(5,2,1);
    plot(t, e); grid on;
    ylabel('e [m]');
    title('Erro lateral');

    subplot(5,2,2);
    plot(t, psi_cont, 'DisplayName', 'heading'); hold on;
    plot(t, psi_ref_cont, '--', 'DisplayName', 'psi_{ref}');
    if has_beta
        plot(t, course_cont, ':', 'LineWidth', 1.5, 'DisplayName', 'course');
    end
    if has_alpha
        plot(t, alpha_cont, '-.', 'DisplayName', 'alpha');
    end
    grid on;
    ylabel('[deg]');
    legend('Location', 'best');
    title('Heading / Course / PsiRef / Alpha');

    subplot(5,2,3);
    psi_err = rad2deg(wrapToPi(psi_rad - psi_ref_rad));
    plot(t, psi_err, 'DisplayName', 'heading - psi_{ref}'); hold on;
    if has_beta
        course_err = rad2deg(wrapToPi(psi_rad + beta - psi_ref_rad));
        plot(t, course_err, ':', 'LineWidth', 1.5, 'DisplayName', 'course - psi_{ref}');
    end
    grid on;
    ylabel('[deg]');
    legend('Location', 'best');
    title('Erro angular');

    subplot(5,2,4);
    plot(t, r_yaw, 'DisplayName', 'r'); hold on;
    plot(t, curvature .* vx_sig, '--', 'DisplayName', 'kappa*vx');
    grid on;
    ylabel('[rad/s]');
    legend('Location', 'best');
    title('Yaw rate vs referencia');

    subplot(5,2,5);
    plot(t, omegam, 'DisplayName', 'omega_m'); hold on;
    plot(t, omegam_ref, '--', 'DisplayName', 'omega_{m,ref}');
    grid on;
    ylabel('[rad/s]');
    legend('Location', 'best');
    title('Velocidade angular motor');

    subplot(5,2,6);
    plot(t, delta_deg); grid on;
    ylabel('[deg]');
    title('Angulo de estercamento');

    subplot(5,2,7);
    plot(t, curvature); grid on;
    ylabel('\kappa [1/m]');
    title('Curvatura');

    subplot(5,2,8);
    plot(t, vx_sig); grid on;
    ylabel('v_x [m/s]');
    title('Velocidade longitudinal');

    if has_beta
        Lr = 1.0;  % default, sera sobrescrito se params disponivel
        try
            p = load('param_MF6713.mat');
            Lr = p.Lr;
        catch
        end

        subplot(5,2,9);
        plot(t, rad2deg(beta), 'DisplayName', 'beta real'); hold on;
        beta_kin = atan(Lr * curvature);
        beta_gyro = atan(Lr * r_yaw ./ vx_sig);
        plot(t, rad2deg(beta_kin), '--', 'DisplayName', 'atan(Lr*kappa)');
        plot(t, rad2deg(beta_gyro), ':', 'LineWidth', 1.5, 'DisplayName', 'atan(Lr*r/vx)');
        grid on;
        ylabel('[deg]');
        legend('Location', 'best');
        title('Sideslip');

        subplot(5,2,10);
        vy_real = vx_sig .* tan(beta);
        plot(t, vy_real); grid on;
        ylabel('v_y [m/s]');
        title('Velocidade lateral');
    end

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

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end
