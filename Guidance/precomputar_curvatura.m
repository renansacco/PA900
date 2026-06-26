function guia = precomputar_curvatura(wps, wpDist, fc)
% PRECOMPUTAR_CURVATURA  Calcula curvatura suavizada do path B-spline.
%
%   guia = precomputar_curvatura(wps, wpDist)
%   guia = precomputar_curvatura(wps, wpDist, fc)
%
%   Avalia o B-spline, extrai alpha(s), filtra com Butterworth 2a ordem
%   fase zero, e computa kappa = d(alpha_filt)/ds.
%
%   Input:
%     wps    — struct com .x, .y (waypoints originais, NAO reamostrados)
%     wpDist — espacamento para resample [m]
%     fc     — frequencia de corte espacial [ciclos/m] (default: 0.10)
%
%   Output:
%     guia.wps       — waypoints reamostrados
%     guia.s         — arclength [m] (Nx1)
%     guia.alpha     — heading filtrado [rad] (Nx1)
%     guia.kappa     — curvatura filtrada [1/m] (Nx1)
%     guia.s_total   — comprimento total do path [m]
%     guia.fc        — frequencia de corte usada [ciclos/m]
%     guia.wpDist    — espacamento de waypoints [m]

if nargin < 3, fc = 0.10; end

%% Resample + B-spline
wps_r = resample_waypoints(wps, wpDist);

npts_seg = 200;
[sx, sy] = avaliar_bspline(wps_r, npts_seg);

%% alpha(s) uniforme em arclength
dx = diff(sx);  dy = diff(sy);
ds_raw = sqrt(dx.^2 + dy.^2);
s_raw  = [0; cumsum(ds_raw)];
s_mid  = s_raw(1:end-1) + ds_raw/2;
alpha_raw = unwrap(atan2(dy, dx));

ds_uni = 0.05;
s_uni = (0:ds_uni:s_mid(end))';
alpha_uni = interp1(s_mid, alpha_raw, s_uni, 'pchip');

%% Butterworth 2a ordem, fase zero
fs = 1 / ds_uni;          % freq de amostragem espacial [ciclos/m]
Wn = fc / (fs/2);         % freq normalizada
[b, a] = butter(2, Wn);
alpha_filt = filtfilt(b, a, alpha_uni);

%% Curvatura = d(alpha_filt)/ds
kappa_filt = gradient(alpha_filt, ds_uni);

%% Reamostra para espaçamento de waypoint (lookup mais leve)
ds_out = wpDist;
s_out = (0:ds_out:s_uni(end))';
alpha_out = interp1(s_uni, alpha_filt, s_out, 'pchip');
kappa_out = interp1(s_uni, kappa_filt, s_out, 'pchip');

%% Monta saida
guia.wps     = wps_r;
guia.s       = s_out;
guia.alpha   = alpha_out;
guia.kappa   = kappa_out;
guia.s_total = s_out(end);
guia.fc      = fc;
guia.wpDist  = wpDist;

end
