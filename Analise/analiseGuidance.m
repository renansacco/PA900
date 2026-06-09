clear; close all;

load(fullfile(fileparts(mfilename('fullpath')), 'logs', 'PA900_TESTE_sess3_14h15m45s.mat'), 'data');

% --- constantes ---
V_OK = 1.0;     % m/s

% --- sinais ---
t       = data.time(:);
e_lat   = data.Guidance.lateralError(:);       % m
e_psi   = data.Guidance.psiError(:);           % rad
alpha   = data.Guidance.alpha(:);              % rad (heading da linha)
kappa   = data.Guidance.curvature(:);          % 1/m
lineIdx = data.Guidance.lineIndex(:);
ap      = data.Control.isAutopilotOn(:) > 0;
speed   = data.GNSS.speed(:) / 3.6;            % km/h -> m/s

% --- mascara base: autopilot on, velocidade util, linha valida (alpha != 0) ---
mask_base = ap & speed > V_OK & alpha ~= 0;

% --- derivada numerica por trecho contiguo de lineIndex ---
% evita que gradient atravesse descontinuidades (troca de linha gera salto
% em lateralError, que vira spike artificial em dE/dt).
elat_dot = nan(size(e_lat));
N = numel(t);
i = 1;
while i <= N
    j = i;
    while j < N && lineIdx(j+1) == lineIdx(i) && alpha(j+1) ~= 0 && alpha(i) ~= 0
        j = j + 1;
    end
    if j - i >= 2
        idx = i:j;
        elat_dot(idx) = gradient(e_lat(idx), t(idx));
    end
    i = j + 1;
end

% --- mascara final: base + derivada valida + exclui bordas (+-1 amostra) ---
valid_deriv = ~isnan(elat_dot);
% exclui tambem a amostra antes e depois de uma troca de lineIdx,
% porque gradient nas pontas usa fwd/bwd diff e fica ruidoso
edge = false(size(t));
edge(2:end)   = edge(2:end)   | (lineIdx(2:end)   ~= lineIdx(1:end-1));
edge(1:end-1) = edge(1:end-1) | (lineIdx(2:end)   ~= lineIdx(1:end-1));

mask = mask_base & valid_deriv & ~edge;

fprintf('V_OK=%.1f m/s\n', V_OK);
fprintf('amostras autopilot on        : %d / %d\n', sum(ap), numel(ap));
fprintf('amostras mascara base        : %d / %d\n', sum(mask_base), numel(mask_base));
fprintf('amostras mascara cinematica  : %d / %d (remove bordas de troca de linha)\n', sum(mask), numel(mask));
fprintf('curvatura (kappa):  min=%+.2e  max=%+.2e  mean=%+.2e  (retas -> ~0)\n', ...
        min(kappa), max(kappa), mean(kappa));
fprintf('trechos de lineIndex:\n');
for li = unique(lineIdx(mask))'
    n = sum(mask & lineIdx==li);
    fprintf('  line=%2d: n=%d\n', li, n);
end

% --- modelo cinematico (curvature ~ 0, so retas): dE/dt = +- v * sin(e_psi) ---
model_pos =  speed .* sin(e_psi);
model_neg = -speed .* sin(e_psi);

% --- testa convencao de sinal via correlacao na mascara ---
c_pos = corr(elat_dot(mask), model_pos(mask));
c_neg = corr(elat_dot(mask), model_neg(mask));
fprintf('\ncorrelacao d(eLat)/dt  vs  +v*sin(ePsi) : %+.3f\n', c_pos);
fprintf('correlacao d(eLat)/dt  vs  -v*sin(ePsi) : %+.3f\n', c_neg);
if abs(c_pos) >= abs(c_neg)
    modelo = model_pos;    sinal = '+';    c_best = c_pos;
else
    modelo = model_neg;    sinal = '-';    c_best = c_neg;
end
fprintf('convencao escolhida: d(eLat)/dt = %sv*sin(ePsi)   (corr=%+.3f)\n', sinal, c_best);

% residuo
resid = elat_dot - modelo;
resid(~mask) = NaN;

fprintf('\nresiduo (na mascara): mean=%+.4f  std=%.4f  max|.|=%.4f  m/s\n', ...
        mean(resid(mask)), std(resid(mask)), max(abs(resid(mask))));

% --- plots ---
elat_dot_plot = elat_dot;   elat_dot_plot(~mask) = NaN;
modelo_plot   = modelo;     modelo_plot(~mask)   = NaN;

figure('Name','Analise Guidance (coerencia cinematica)','Color','w');

ax1 = subplot(4,1,1);
yyaxis left;  plot(t, e_psi, 'LineWidth', 1); ylabel('psiError [rad]'); grid on;
yyaxis right; plot(t, rad2deg(e_psi), 'LineStyle','none');              % so pra dar eixo em deg
ylabel('psiError [deg]');
title('psiError');

ax2 = subplot(4,1,2);
yyaxis left;  plot(t, e_lat,  'LineWidth', 1); ylabel('lateralError [m]'); grid on;
yyaxis right; plot(t, speed,  'LineWidth', 1); ylabel('speed [m/s]');
title('lateralError e velocidade');

ax3 = subplot(4,1,3);
plot(t, elat_dot_plot, 'LineWidth', 1); hold on; grid on;
plot(t, modelo_plot,   'LineWidth', 1);
ylabel('dE_{lat}/dt [m/s]');
legend('medido (grad. num.)', sprintf('modelo %sv sin(ePsi)', sinal), 'Location','best');
title('Coerencia cinematica (retas: d(eLat)/dt = +- v sin(ePsi))');

ax4 = subplot(4,1,4);
yyaxis left;  plot(t, resid,   'LineWidth', 1); ylabel('residuo [m/s]'); grid on;
yyaxis right; plot(t, lineIdx, 'LineWidth', 1); ylabel('lineIndex');
xlabel('t [s]');
title('Residuo e linha ativa');

linkaxes([ax1 ax2 ax3 ax4], 'x');
