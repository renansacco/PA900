function guia = condicionar_guia(wps_original, wpDist, npts, fc, ordem)
% CONDICIONAR_GUIA  Condiciona waypoints em struct guia padronizada.
%
%   guia = condicionar_guia(wps_original)
%   guia = condicionar_guia(wps_original, wpDist, npts, fc, ordem)
%
%   Input:
%       wps_original — struct com .x e .y (waypoints brutos)
%       wpDist       — distancia de resample [m] (default: 3.0)
%       npts         — pontos por segmento (default: 50)
%       fc           — freq corte do Butterworth p/ kappa_smooth [ciclos/m] (default: 0.10)
%       ordem        — ordem do Butterworth (default: 2)
%
%   Output:
%       guia.s            — arclength [m]
%       guia.x            — posicao x [m]
%       guia.y            — posicao y [m]
%       guia.alpha        — heading da spline [rad]
%       guia.kappa        — curvatura [1/m]
%       guia.kappa_smooth — curvatura filtrada [1/m]
%       guia.dkappa_ds    — derivada da curvatura suavizada em relacao a s [1/m^2]
%       guia.wps          — waypoints resampled (com wp extra no final)
%       guia.wpDist       — distancia de resample usada [m]
%       guia.fc           — freq de corte usada [ciclos/m]
%       guia.filt_ordem   — ordem do filtro

    if nargin < 2, wpDist = 3.0; end
    if nargin < 3, npts = 50; end
    if nargin < 4, fc = 0.10; end
    if nargin < 5, ordem = 2; end

    %% Resample
    wps = resample_waypoints(wps_original, wpDist);
    wx = wps.x(:);
    wy = wps.y(:);

    % Wp extra no final (replica guidance_init)
    n0 = numel(wx);
    dx = wx(n0) - wx(n0-1);
    dy = wy(n0) - wy(n0-1);
    L  = sqrt(dx^2 + dy^2);
    if L > 1e-6
        wx = [wx; wx(n0) + wpDist*dx/L];
        wy = [wy; wy(n0) + wpDist*dy/L];
    end
    n = numel(wx);

    %% Avaliar spline segmento a segmento
    t_vec = linspace(0, 1, npts+1);
    t_vec = t_vec(1:end-1);

    N_total = (n-1)*numel(t_vec) + 1;
    all_x   = zeros(N_total, 1);
    all_y   = zeros(N_total, 1);
    all_xd  = zeros(N_total, 1);
    all_yd  = zeros(N_total, 1);
    all_xdd = zeros(N_total, 1);
    all_ydd = zeros(N_total, 1);

    idx = 0;
    for seg = 2:n
        [xc, yc] = computeSpline(wx, wy, n, seg);
        for k = 1:numel(t_vec)
            idx = idx + 1;
            [all_x(idx), all_y(idx), all_xd(idx), all_yd(idx), ...
             all_xdd(idx), all_ydd(idx)] = evalSpline(xc, yc, t_vec(k));
        end
    end

    % Ponto final t=1 do ultimo segmento
    [xc, yc] = computeSpline(wx, wy, n, n);
    idx = idx + 1;
    [all_x(idx), all_y(idx), all_xd(idx), all_yd(idx), ...
     all_xdd(idx), all_ydd(idx)] = evalSpline(xc, yc, 1.0);

    %% alpha, kappa, s
    alpha = unwrap(atan2(all_yd, all_xd));

    speed2 = all_xd.^2 + all_yd.^2;
    kappa  = (all_xd .* all_ydd - all_yd .* all_xdd) ./ (speed2.^1.5);
    kappa(speed2 < 1e-12) = 0;

    ds = sqrt(diff(all_x).^2 + diff(all_y).^2);
    s  = [0; cumsum(ds)];

    %% kappa_smooth (Butterworth zero-phase)
    ds_med = median(diff(s));
    s_uni  = (s(1) : ds_med : s(end))';
    k_uni  = interp1(s, kappa, s_uni, 'pchip');
    fs     = 1 / ds_med;
    Wn     = min(fc / (fs/2), 0.99);
    [b, a] = butter(ordem, Wn);
    k_filt = filtfilt(b, a, k_uni);
    kappa_smooth = interp1(s_uni, k_filt, s, 'pchip');

    %% dkappa_ds — derivada da curvatura suavizada em relacao a s
     dkappa_ds = gradient(kappa_smooth, s);
% k_filt    = filtfilt(b, a, k_uni);          % kappa filtrada na grade uniforme s_uni
% dk_uni    = gradient(k_filt, ds_med);       % deriva com ?s CONSTANTE -> limpo
% dkappa_ds = interp1(s_uni, dk_uni, s, 'pchip');
    %% Saida
    guia.s            = s;
    guia.x            = all_x;
    guia.y            = all_y;
    guia.alpha        = alpha;
    guia.kappa        = kappa;
    guia.kappa_smooth = kappa_smooth;
    guia.dkappa_ds    = dkappa_ds;
    guia.wps          = struct('x', wx, 'y', wy);
    guia.wpDist       = wpDist;
    guia.fc           = fc;
    guia.filt_ordem   = ordem;
end

% =========================================================================
% Subfuncoes — replica exata do guidance_step
% =========================================================================

function [xc, yc] = computeSpline(wx, wy, n, wpIndex)
    if wpIndex == 1
        P1x = wx(1); P1y = wy(1);
        P2x = wx(2); P2y = wy(2);
        P0x = 2*P1x - P2x;  P0y = 2*P1y - P2y;
        if n >= 3
            P3x = wx(3); P3y = wy(3);
        else
            P3x = 2*P2x - P1x; P3y = 2*P2y - P1y;
        end
    elseif wpIndex >= n
        P2x = wx(n);   P2y = wy(n);
        P1x = wx(n-1); P1y = wy(n-1);
        P3x = 2*P2x - P1x; P3y = 2*P2y - P1y;
        if n >= 3
            P0x = wx(n-2); P0y = wy(n-2);
        else
            P0x = 2*P1x - P2x; P0y = 2*P1y - P2y;
        end
    else
        P1x = wx(wpIndex-1); P1y = wy(wpIndex-1);
        P2x = wx(wpIndex);   P2y = wy(wpIndex);
        if wpIndex - 2 >= 1
            P0x = wx(wpIndex-2); P0y = wy(wpIndex-2);
        else
            P0x = 2*P1x - P2x; P0y = 2*P1y - P2y;
        end
        if wpIndex + 1 <= n
            P3x = wx(wpIndex+1); P3y = wy(wpIndex+1);
        else
            P3x = 2*P2x - P1x; P3y = 2*P2y - P1y;
        end
    end

    s = 1/6;
    Px = [P0x; P1x; P2x; P3x];
    Py = [P0y; P1y; P2y; P3y];
    M = s * [1, -3,  3, -1;
             4,  0, -6,  3;
             1,  3,  3, -3;
             0,  0,  0,  1];
    xc = (M' * Px)';
    yc = (M' * Py)';
end

function [sx, sy, sxd, syd, sxdd, sydd] = evalSpline(xc, yc, t)
    ax = xc(1); bx = xc(2); cx = xc(3); dx = xc(4);
    ay = yc(1); by = yc(2); cy = yc(3); dy = yc(4);
    sx   = ax + bx*t + cx*t^2 + dx*t^3;
    sy   = ay + by*t + cy*t^2 + dy*t^3;
    sxd  = bx + 2*cx*t + 3*dx*t^2;
    syd  = by + 2*cy*t + 3*dy*t^2;
    sxdd = 2*cx + 6*dx*t;
    sydd = 2*cy + 6*dy*t;
end
