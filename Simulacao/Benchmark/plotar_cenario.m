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
    if isfield(r, 'version')
        titulo = sprintf('[%s] %s', r.version, titulo);
    end
    if ~isempty(ctrl_label)
        titulo = sprintf('[%s] %s', ctrl_label, titulo);
    end

    if isfield(r, 'omegam_sat')
        omegam_sat = r.omegam_sat;
    else
        omegam_sat = 15;
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
    if isfield(r, 'wps_original')
        plot(r.wps_original.x, r.wps_original.y, 'k.--', ...
            'DisplayName', 'Waypoints originais'); hold on;
        plot(wps.x, wps.y, 'ms', 'MarkerSize', 4, ...
            'DisplayName', sprintf('Resampled (%.1fm)', r.wpDistance));
    else
        plot(wps.x, wps.y, 'k.--', 'DisplayName', 'Waypoints'); hold on;
    end
    [sx, sy] = avaliar_bspline(wps, 30);
    plot(sx, sy, 'r-', 'LineWidth', 1.5, 'DisplayName', 'B-spline (guidance)');
    plot(xy(:,1), xy(:,2), 'b', 'LineWidth', 1, 'DisplayName', 'Veiculo');
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
    hold on;
    %plot(t, psi_cont, 'DisplayName', 'heading'); 
    plot(t, psi_ref_cont, '--', 'DisplayName', 'psi_{ref}');hold on;
    if has_beta
        plot(t, course_cont, ':', 'LineWidth', 1.5, 'DisplayName', 'course');
    end
    if has_alpha
        %plot(t, alpha_cont, '-.', 'DisplayName', 'alpha');
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
        e_ang_rms = rms(course_err);
        e_ang_max = max(abs(course_err));
    else
        e_ang_rms = rms(psi_err);
        e_ang_max = max(abs(psi_err));
    end
    grid on;
    ylabel('[deg]');
    legend('Location', 'best');
    title(sprintf('Erro angular (RMS=%.1f°, max=%.1f°)', e_ang_rms, e_ang_max));

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
    yline( omegam_sat, 'k:', 'HandleVisibility', 'off');
    yline(-omegam_sat, 'k:', 'HandleVisibility', 'off');
    grid on;
    ylabel('[rad/s]');
    legend('Location', 'best');
    title(sprintf('Velocidade angular motor (sat=%.0f)', omegam_sat));

    subplot(5,2,6);
    plot(t, delta_deg); grid on;
    ylabel('[deg]');
    title('Angulo de estercamento');

    subplot(5,2,7);
    plot(t, curvature, 'DisplayName', '\kappa guidance'); hold on;
    if isfield(r, 'guia')
        s_approx = cumtrapz(t, vx_sig);
        kappa_filt = interp1(r.guia.s, r.guia.kappa, s_approx, 'pchip', 0);
        plot(t, kappa_filt, 'r--', 'LineWidth', 1.2, 'DisplayName', ...
            sprintf('\\kappa filtrada (fc=%.2f)', r.guia.fc));
        legend('Location', 'best');
    end
    grid on;
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

    %% Resumo no console
    fprintf('\n=== Resumo: %s ===\n', titulo);
    fprintf('  vx:              %.1f m/s\n', mean(vx_sig));
    fprintf('  e_lat mean:      %.1f cm\n', mean(abs(e))*100);
    fprintf('  e_lat max:       %.1f cm\n', max(abs(e))*100);
    fprintf('  e_lat rms:       %.1f cm\n', rms(e)*100);
    fprintf('  e_psi rms:       %.2f deg\n', e_ang_rms);
    fprintf('  e_psi max:       %.2f deg\n', e_ang_max);
    fprintf('  omega_m max:     %.1f rad/s  (sat=%.0f)\n', max(abs(omegam_ref)), omegam_sat);
    fprintf('  delta max:       %.1f deg\n', max(abs(delta_deg)));
    fprintf('  sat omega_m:     %.1f%%  (|omega| >= %.0f*0.99)\n', ...
        100*sum(abs(omegam_ref) >= omegam_sat * 0.99)/length(t), omegam_sat);
end

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end