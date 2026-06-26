%% Analise em frequencia da malha fechada — pos-projeto
%
% Usa os ganhos K_psi, K_r do Projeto_Curva_Linear para fechar a malha,
% e analisa o Bode MF (psi_ref -> psi). Extrai banda passante e tau_MF.
% Compara tau_MF com tau_la = Delta/vx para avaliar Delta viavel.
%
% Rodar APOS Projeto_Curva_Linear.m (usa variaveis do workspace).

%% Parametros
proj_root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
addpath(fullfile(proj_root, 'Planta'));
p = load(fullfile(proj_root, 'Planta', 'params', 'param_MF6713.mat'));

vx_list    = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0];
Delta_list = [2, 3, 4, 5, 6, 8];
omega_sat  = 10;
colors     = lines(length(vx_list));
w          = logspace(-2, 2, 500);

% Ganhos — usar os otimizados (ajustar manualmente ou carregar)
% Por ora, usa ganhos fixos do ultimo projeto (vx=2, IT2AE)
K_psi_nom = 64.06;
K_r_nom   = 67.86;

%% Lineariza e fecha malha para cada vx
mf_sys = cell(length(vx_list), 1);
bw     = zeros(length(vx_list), 1);
tau_mf = zeros(length(vx_list), 1);

for k = 1:length(vx_list)
    vx = vx_list(k);
    Xe = zeros(7, 1);
    Ue = [0; vx];
    [A, B] = linearizar_veiculo(Xe, Ue, p);

    % Reduz para estados dinamicos: [psi r vy omegam delta] ou [psi r vy delta omegam]
    idx = 3:7;
    Ar = A(idx, idx);
    Br = B(idx, 1);

    C_psi = [1 0 0 0 0];
    C_r   = [0 1 0 0 0];

    % Malha fechada: u = K_psi*(psi_ref - psi) + K_r*(0 - r)
    %   u = K_psi*psi_ref - K_psi*psi - K_r*r
    %   u = K_psi*psi_ref - [K_psi  K_r  0  0  0]*X_r
    K_fb = [K_psi_nom, K_r_nom, 0, 0, 0];

    A_mf = Ar - Br * K_fb;
    B_mf = Br * K_psi_nom;    % entrada = psi_ref
    C_mf = C_psi;
    D_mf = 0;

    sys_mf = ss(A_mf, B_mf, C_mf, D_mf);
    mf_sys{k} = sys_mf;

    % Banda passante (-3dB)
    [mag_vec, ~] = bode(sys_mf, w);
    mag_dB = 20*log10(squeeze(mag_vec));
    mag0 = mag_dB(1);   % ganho DC
    idx_bw = find(mag_dB < mag0 - 3, 1, 'first');
    if ~isempty(idx_bw)
        bw(k) = w(idx_bw);
    else
        bw(k) = w(end);
    end
    tau_mf(k) = 1 / bw(k);
end

%% ============================================================
%%  FIGURA 1 — Bode MF: psi_ref -> psi
%% ============================================================
figure('Name', 'Bode MF');

subplot(2,1,1); hold on;
for k = 1:length(vx_list)
    [mag, ~] = bode(mf_sys{k}, w);
    semilogx(w, 20*log10(squeeze(mag)), 'Color', colors(k,:), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('vx=%.1f', vx_list(k)));
end
yline(-3, 'k--', '-3dB', 'HandleVisibility', 'off');
ylabel('|T(j\omega)| [dB]'); grid on;
legend('Location', 'southwest');
title('Magnitude');
set(gca, 'XScale', 'log');

subplot(2,1,2); hold on;
for k = 1:length(vx_list)
    [~, phase] = bode(mf_sys{k}, w);
    semilogx(w, squeeze(phase), 'Color', colors(k,:), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('vx=%.1f', vx_list(k)));
end
ylabel('Fase [deg]'); xlabel('\omega [rad/s]'); grid on;
legend('Location', 'southwest');
title('Fase');
set(gca, 'XScale', 'log');

sgtitle(sprintf('Malha fechada: \\psi_{ref} \\rightarrow \\psi   (K_{\\psi}=%.1f, K_r=%.1f)', ...
    K_psi_nom, K_r_nom));

%% ============================================================
%%  FIGURA 2 — Bandwidth e tau_MF vs vx
%% ============================================================
figure('Name', 'Bandwidth e tau_MF');

subplot(2,1,1);
plot(vx_list, bw, 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
ylabel('\omega_{bw} [rad/s]'); xlabel('v_x [m/s]'); grid on;
title('Banda passante da malha fechada (-3dB)');

subplot(2,1,2); hold on;
plot(vx_list, tau_mf, 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k', ...
    'DisplayName', '\tau_{MF} = 1/\omega_{bw}');
for d = 1:length(Delta_list)
    Delta = Delta_list(d);
    plot(vx_list, Delta ./ vx_list, '--', 'LineWidth', 1.2, ...
        'DisplayName', sprintf('\\tau_{la} (\\Delta=%dm)', Delta));
end
ylabel('\tau [s]'); xlabel('v_x [m/s]'); grid on;
legend('Location', 'best');
title('\tau_{MF} vs \tau_{la} = \Delta/v_x');

sgtitle('Limites da malha fechada vs exigencia do lookahead');

%% ============================================================
%%  FIGURA 3 — omega_la / omega_bw  (quanto da banda o lookahead exige)
%% ============================================================
figure('Name', 'Viabilidade de Delta');
hold on;
for d = 1:length(Delta_list)
    Delta = Delta_list(d);
    omega_la = vx_list / Delta;
    ratio = omega_la ./ bw';
    plot(vx_list, ratio, 'o-', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('\\Delta=%.0fm', Delta));
end
yline(1, 'r--', '\omega_{la} = \omega_{bw}', 'HandleVisibility', 'off', 'LineWidth', 1.5);
yline(0.5, 'r:', '\omega_{la} = 0.5·\omega_{bw}', 'HandleVisibility', 'off');
ylabel('\omega_{la} / \omega_{bw}'); xlabel('v_x [m/s]'); grid on;
legend('Location', 'best');
title('\omega_{la}/\omega_{bw}: < 0.5 = confortavel,  > 1 = impossivel');
sgtitle(sprintf('Viabilidade de \\Delta  (K_{\\psi}=%.1f, K_r=%.1f)', K_psi_nom, K_r_nom));

%% ============================================================
%%  Tabela resumo
%% ============================================================
fprintf('\n=== Banda passante da malha fechada ===\n');
fprintf('%5s  %8s  %8s\n', 'vx', 'w_bw', 'tau_MF');
for k = 1:length(vx_list)
    fprintf('%5.1f  %8.2f  %8.2f\n', vx_list(k), bw(k), tau_mf(k));
end

fprintf('\n=== omega_la / omega_bw  (< 0.5 = ok, > 1 = inviavel) ===\n');
fprintf('%5s', 'vx');
for d = 1:length(Delta_list)
    fprintf('  D=%-4.0f', Delta_list(d));
end
fprintf('\n');
for k = 1:length(vx_list)
    fprintf('%5.1f', vx_list(k));
    for d = 1:length(Delta_list)
        omega_la = vx_list(k) / Delta_list(d);
        fprintf('  %5.2f', omega_la / bw(k));
    end
    fprintf('\n');
end
fprintf('\nCriterio: omega_la/omega_bw < 0.5 => rastreio confortavel\n');
fprintf('          omega_la/omega_bw > 1.0 => malha nao consegue acompanhar\n');
