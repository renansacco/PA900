%% Analise da dinamica do lookahead: nao-linear vs linearizado
%
% Derivacao:
%   O lookahead gera psi_ref = psi_path + gamma, com gamma = atan(-e/Delta).
%   Assumindo rastreio perfeito (psi = psi_ref), o erro lateral evolui como:
%     de/dt = v * sin(gamma) = v * sin(atan(-e/Delta)) = -v*e / sqrt(Delta^2 + e^2)
%
%   Linearizando (e << Delta):
%     de/dt ≈ -(v/Delta) * e  =>  e(t) = e0 * exp(-v*t/Delta)
%     gamma(t) ≈ -(e0/Delta) * exp(-v*t/Delta)
%     Constante de tempo: tau = Delta/v
%
% Conclusao: a linearizacao casa bem para e0/Delta < ~0.5.

clear; close all;

%% Parametros
Delta = 4.0;                        % lookahead [m]
vx_list = [1.0, 2.0, 3.0];         % velocidades [m/s]
e0_list = [0.5, 1.0, 2.0, 4.0];    % erro lateral inicial [m]
tsim = 15;                          % tempo de simulacao [s]

%% Figura 1: variando e0, vx fixo
vx = 2.0;
figure('Name', 'Variando e0');

for k = 1:length(e0_list)
    e0 = e0_list(k);

    % Nao-linear
    [t_nl, e_nl] = ode45(@(t,e) -vx*e/sqrt(Delta^2 + e^2), [0 tsim], e0);
    gamma_nl = atan(-e_nl / Delta);

    % Linearizado
    tau = Delta / vx;
    e_lin = e0 * exp(-t_nl / tau);
    gamma_lin = -e_lin / Delta;

    subplot(2, length(e0_list), k);
    plot(t_nl, e_nl, 'b', 'LineWidth', 1.5); hold on;
    plot(t_nl, e_lin, 'r--', 'LineWidth', 1.5);
    ylabel('e [m]'); grid on;
    title(sprintf('e_0 = %.1f m', e0));
    if k == 1, legend('nao-linear', 'linearizado', 'Location', 'best'); end

    subplot(2, length(e0_list), k + length(e0_list));
    plot(t_nl, rad2deg(gamma_nl), 'b', 'LineWidth', 1.5); hold on;
    plot(t_nl, rad2deg(gamma_lin), 'r--', 'LineWidth', 1.5);
    ylabel('\gamma [deg]'); xlabel('t [s]'); grid on;
end
sgtitle(sprintf('vx = %.1f m/s,  \\Delta = %.1f m,  \\tau = %.1f s', vx, Delta, Delta/vx));

%% Figura 2: variando vx, e0 fixo
e0 = 1.0;
figure('Name', 'Variando vx');

for k = 1:length(vx_list)
    vx = vx_list(k);

    [t_nl, e_nl] = ode45(@(t,e) -vx*e/sqrt(Delta^2 + e^2), [0 tsim], e0);
    gamma_nl = atan(-e_nl / Delta);

    tau = Delta / vx;
    e_lin = e0 * exp(-t_nl / tau);
    gamma_lin = -e_lin / Delta;

    subplot(2, length(vx_list), k);
    plot(t_nl, e_nl, 'b', 'LineWidth', 1.5); hold on;
    plot(t_nl, e_lin, 'r--', 'LineWidth', 1.5);
    ylabel('e [m]'); grid on;
    title(sprintf('vx = %.1f m/s  (\\tau = %.1f s)', vx, tau));
    if k == 1, legend('nao-linear', 'linearizado', 'Location', 'best'); end

    subplot(2, length(vx_list), k + length(vx_list));
    plot(t_nl, rad2deg(gamma_nl), 'b', 'LineWidth', 1.5); hold on;
    plot(t_nl, rad2deg(gamma_lin), 'r--', 'LineWidth', 1.5);
    ylabel('\gamma [deg]'); xlabel('t [s]'); grid on;
end
sgtitle(sprintf('e_0 = %.1f m,  \\Delta = %.1f m', e0, Delta));

%% Figura 3: erro relativo da linearizacao
figure('Name', 'Erro da linearizacao');
for k = 1:length(e0_list)
    e0 = e0_list(k);
    vx = 2.0;

    [t_nl, e_nl] = ode45(@(t,e) -vx*e/sqrt(Delta^2 + e^2), [0 tsim], e0);
    tau = Delta / vx;
    e_lin = e0 * exp(-t_nl / tau);

    err_pct = 100 * (e_lin - e_nl) ./ max(e_nl, 1e-6);

    subplot(1, length(e0_list), k);
    plot(t_nl, err_pct, 'LineWidth', 1.5);
    ylabel('erro [%]'); xlabel('t [s]'); grid on;
    title(sprintf('e_0 = %.1f m  (e_0/\\Delta = %.1f)', e0, e0/Delta));
end
sgtitle('Erro relativo: (linearizado - nao-linear) / nao-linear');
