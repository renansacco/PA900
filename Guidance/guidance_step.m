function [out, state] = guidance_step(pose, state)
% GUIDANCE_STEP  Calcula erros de guiagem via B-spline (replica do C++).
%
%   [out, state] = guidance_step(pose, state)
%
%   Input:
%       pose  — [x, y, psi] posicao e heading do veiculo [m, m, rad]
%       state — struct retornada por guidance_init
%
%   Output:
%       out.lateralError — erro lateral [m] (+ = veiculo a esquerda)
%       out.psiError     — erro angular [rad] (in (-pi, pi])
%       out.alpha        — heading da spline no ponto mais proximo [rad]
%       out.curvature    — curvatura da spline [1/m]
%       out.lineIndex    — indice do segmento ativo (wpIndex)
%       out.status       — 0=Inactive, 1=Active, 2=EndOfPath, 3=Diverged
%       out.t            — parametro da biseccao [0,1]
%
%   Algoritmo:
%   1. Busca do segmento mais proximo (distancia ponto-segmento)
%   2. Calculo dos coeficientes B-spline cubico uniforme
%   3. Biseccao do polinomio de 5o grau (derivada da dist^2) em [-0.2, 1.2]
%   4. Calculo de alpha, erro lateral, psiError, curvatura via derivadas da spline
%   5. Avanco de waypoint e deteccao de divergencia
%
%   Data: 2026-06-09 | Autor: Renan / Claude

    % Default output
    out.lateralError = 0;
    out.psiError     = 0;
    out.alpha        = 0;
    out.curvature    = 0;
    out.lineIndex    = 0;
    out.status       = state.status;
    out.t            = 0;

    % Early exit: no path or terminal state
    if ~state.isReady || state.status == 0
        return;
    end
    if state.status == 2 || state.status == 3  % EndOfPath or Diverged
        alpha = state.lastAlpha;
        px = pose(1); py = pose(2); psi = pose(3);
        wx = state.wps.x(:); wy = state.wps.y(:);
        ex = px - wx(end); ey = py - wy(end);
        out.lateralError = -ex*sin(alpha) + ey*cos(alpha);
        out.psiError     = wrapToPi(psi - alpha);
        out.alpha        = alpha;
        out.curvature    = 0;
        out.lineIndex    = state.wpIndex;
        out.status       = state.status;
        return;
    end

    wx = state.wps.x(:);
    wy = state.wps.y(:);
    n  = state.n;
    px = pose(1);
    py = pose(2);
    psi = pose(3);

    % --- 1. Find closest segment (on first call or if spline invalid) ---
    if state.wpIndex == 0
        state.wpIndex = findClosestSegment(wx, wy, px, py, state.cfg.searchRadius);
        if state.wpIndex <= 0
            state.status = 3;  % Diverged (can't find path)
            out.status = 3;
            return;
        end
        state.splineValid = false;
    end

    % --- 2. Compute B-spline coefficients if needed ---
    if ~state.splineValid
        [state.xCoefs, state.yCoefs, ok] = computeSpline(wx, wy, n, state.wpIndex);
        if ~ok
            state.status = 2;  % EndOfPath
            out.status = 2;
            return;
        end
        state.splineValid = true;
    end

    % --- 3. Project pose onto spline (bisection) ---
    [t, divFlag] = projectOnSpline(px, py, state.xCoefs, state.yCoefs);
    state.lastT = t;

    if divFlag
        state.countDiv = state.countDiv + 1;
    else
        state.countDiv = 0;
    end

    % --- 4. Evaluate spline and derivatives at t ---
    ax = state.xCoefs(1); bx = state.xCoefs(2); cx = state.xCoefs(3); dx = state.xCoefs(4);
    ay = state.yCoefs(1); by = state.yCoefs(2); cy = state.yCoefs(3); dy = state.yCoefs(4);

    % S(t)
    Sx = ax + bx*t + cx*t^2 + dx*t^3;
    Sy = ay + by*t + cy*t^2 + dy*t^3;

    % S'(t)
    Sx_d = bx + 2*cx*t + 3*dx*t^2;
    Sy_d = by + 2*cy*t + 3*dy*t^2;

    % S''(t)
    Sx_dd = 2*cx + 6*dx*t;
    Sy_dd = 2*cy + 6*dy*t;

    % Alpha (heading da spline)
    alpha = atan2(Sy_d, Sx_d);

    % Lateral error
    lateralError = -(px - Sx)*sin(alpha) + (py - Sy)*cos(alpha);

    % Heading error (shortest angular distance)
    psiError = wrapToPi(psi - alpha);

    % Curvature: kappa = (x'*y'' - y'*x'') / |S'|^3
    speed2 = Sx_d^2 + Sy_d^2;
    if speed2 > 1e-12
        curvature = (Sx_d*Sy_dd - Sy_d*Sx_dd) / (speed2^1.5);
    else
        curvature = 0;
    end

    % --- 5. Waypoint advancement ---
    if t >= 1.0
        % Distance to end of current segment
        endX = ax + bx + cx + dx;  % S(1)
        endY = ay + by + cy + dy;
        distEnd = sqrt((px - endX)^2 + (py - endY)^2);

        if distEnd < state.cfg.wpDistance / 2
            if state.wpIndex < n - 1
                state.wpIndex = state.wpIndex + 1;
                state.splineValid = false;
            else
                state.status = 2;  % EndOfPath
            end
        end
    end

    % --- 6. Divergence detection ---
    if state.countDiv > state.cfg.divergenceCountMax || abs(psiError) > state.cfg.divergencePsiMax
        state.status = 3;  % Diverged
    end

    % --- Output ---
    state.lastAlpha  = alpha;
    out.lateralError = lateralError;
    out.psiError     = psiError;
    out.alpha        = alpha;
    out.curvature    = curvature;
    out.lineIndex    = state.wpIndex;
    out.status       = state.status;
    out.t            = t;
end

% =========================================================================
% SUBFUNCTIONS
% =========================================================================

function idx = findClosestSegment(wx, wy, px, py, radius)
% Find closest segment endpoint within search radius.
% Returns endpoint index (1-based, >=2), or 0 if none found.
    n = numel(wx);
    r2 = radius^2;

    % Mark waypoints within radius
    inRadius = false(n, 1);
    for i = 1:n
        if (px - wx(i))^2 + (py - wy(i))^2 <= r2
            inRadius(i) = true;
        end
    end

    % Evaluate segments adjacent to marked points
    bestDist = inf;
    idx = 0;
    for i = 1:n
        if ~inRadius(i), continue; end

        % Check segment [i-1, i]
        if i >= 2
            d = segmentDist(px, py, wx(i-1), wy(i-1), wx(i), wy(i));
            if d < bestDist
                bestDist = d;
                idx = i;  % endpoint of segment
            end
        end
        % Check segment [i, i+1]
        if i <= n-1
            d = segmentDist(px, py, wx(i), wy(i), wx(i+1), wy(i+1));
            if d < bestDist
                bestDist = d;
                idx = i + 1;  % endpoint of segment
            end
        end
    end
end

function d = segmentDist(px, py, ax, ay, bx, by)
% Perpendicular distance from point to line segment [A, B], clamped.
    abx = bx - ax;
    aby = by - ay;
    apx = px - ax;
    apy = py - ay;
    len2 = abx^2 + aby^2;
    if len2 < 1e-12
        d = sqrt(apx^2 + apy^2);
        return;
    end
    t = (apx*abx + apy*aby) / len2;
    t = max(0, min(1, t));
    cx = ax + t*abx;
    cy = ay + t*aby;
    d = sqrt((px - cx)^2 + (py - cy)^2);
end

function [xc, yc, ok] = computeSpline(wx, wy, n, wpIndex)
% Compute uniform cubic B-spline coefficients for segment ending at wpIndex.
%
% B-spline basis matrix (uniform):
%   M = (1/6) * [ 1  -3   3  -1 ;
%                  4   0  -6   3 ;
%                  1   3   3  -3 ;
%                  0   0   0   1 ]
%
% S(t) = [1, t, t^2, t^3] * M * [P0; P1; P2; P3]
%      = a + b*t + c*t^2 + d*t^3

    ok = true;

    % Select 4 control points with phantom mirroring at boundaries
    % For segment ending at wpIndex, control points are
    % [wpIndex-2, wpIndex-1, wpIndex, wpIndex+1]
    if wpIndex == 1
        % First segment: phantom P0
        P1x = wx(1); P1y = wy(1);
        P2x = wx(2); P2y = wy(2);
        P0x = 2*P1x - P2x;  P0y = 2*P1y - P2y;
        if n >= 3
            P3x = wx(3); P3y = wy(3);
        else
            P3x = 2*P2x - P1x; P3y = 2*P2y - P1y;
        end
    elseif wpIndex >= n
        % Last segment: phantom P3
        if wpIndex > n
            ok = false;
            xc = zeros(1,4); yc = zeros(1,4);
            return;
        end
        P2x = wx(n);   P2y = wy(n);
        P1x = wx(n-1); P1y = wy(n-1);
        P3x = 2*P2x - P1x; P3y = 2*P2y - P1y;
        if n >= 3
            P0x = wx(n-2); P0y = wy(n-2);
        else
            P0x = 2*P1x - P2x; P0y = 2*P1y - P2y;
        end
    else
        % Middle segment: control points [wpIndex-2, wpIndex-1, wpIndex, wpIndex+1]
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

    % B-spline basis matrix coefficients
    % M = (1/6) * [1 -3  3 -1; 4  0 -6  3; 1  3  3 -3; 0  0  0  1]
    % coefs = M * P  (each row of M applied to [P0, P1, P2, P3])
    % Result: S(t) = a + b*t + c*t^2 + d*t^3

    s = 1/6;
    Px = [P0x; P1x; P2x; P3x];
    Py = [P0y; P1y; P2y; P3y];

    M = s * [1, -3,  3, -1;
             4,  0, -6,  3;
             1,  3,  3, -3;
             0,  0,  0,  1];

    % coeffs = M' * P gives [a; b; c; d]
    % S(t) = [1 t t^2 t^3] * [a; b; c; d]
    cx_vec = M' * Px;
    cy_vec = M' * Py;

    xc = cx_vec';  % [a, b, c, d]
    yc = cy_vec';
end

function [t, divFlag] = projectOnSpline(px, py, xc, yc)
% Bisection search for closest point on spline to (px, py).
% Solves d/dt |S(t) - pose|^2 = 0  (5th degree polynomial).

    ax = xc(1); bx = xc(2); cx = xc(3); dx = xc(4);
    ay = yc(1); by = yc(2); cy = yc(3); dy = yc(4);

    % Build 5th-degree polynomial coefficients of f'(t) = d/dt |S(t)-P|^2
    % f(t) = (ax + bx*t + cx*t^2 + dx*t^3 - px)^2 + (ay + by*t + cy*t^2 + dy*t^3 - py)^2
    % f'(t) = 2*(Sx-px)*Sx' + 2*(Sy-py)*Sy'
    % Expanding: sum of products of polynomials

    % Let ex = ax - px, ey = ay - py  (shift origin)
    ex = ax - px;
    ey = ay - py;

    % Sx(t) - px = ex + bx*t + cx*t^2 + dx*t^3
    % Sx'(t)     = bx + 2*cx*t + 3*dx*t^2
    % f'(t)/2    = (ex + bx*t + cx*t^2 + dx*t^3)*(bx + 2*cx*t + 3*dx*t^2)
    %            + (ey + by*t + cy*t^2 + dy*t^3)*(by + 2*cy*t + 3*dy*t^2)

    % Multiply and collect by power of t (degree 0 to 5):
    P = zeros(1, 6);  % P(1) = coef of t^0, ..., P(6) = coef of t^5

    % t^0: ex*bx + ey*by
    P(1) = ex*bx + ey*by;

    % t^1: 2*ex*cx + bx^2 + 2*ey*cy + by^2
    P(2) = 2*ex*cx + bx^2 + 2*ey*cy + by^2;

    % t^2: 3*ex*dx + 3*bx*cx + 3*ey*dy + 3*by*cy
    P(3) = 3*ex*dx + 3*bx*cx + 3*ey*dy + 3*by*cy;

    % t^3: 4*bx*dx + 2*cx^2 + 4*by*dy + 2*cy^2
    P(4) = 4*bx*dx + 2*cx^2 + 4*by*dy + 2*cy^2;

    % t^4: 5*cx*dx + 5*cy*dy
    P(5) = 5*cx*dx + 5*cy*dy;

    % t^5: 3*dx^2 + 3*dy^2
    P(6) = 3*dx^2 + 3*dy^2;

    % Bisection on [-0.2, 1.2]
    a_val = -0.2;
    b_val =  1.2;
    divFlag = false;
    MAX_ITER = 20;

    for iter = 1:MAX_ITER
        t_mid = (a_val + b_val) / 2;

        % Evaluate polynomial via Horner
        fa = evalPoly(P, a_val);
        fc = evalPoly(P, t_mid);

        % Convergence
        if abs(fc) < 0.001 && (b_val - a_val)/2 < 0.001
            break;
        end

        if sign(fa) == sign(fc)
            a_val = t_mid;
        else
            b_val = t_mid;
        end

        if iter == MAX_ITER
            divFlag = true;
        end
    end

    t = (a_val + b_val) / 2;
end

function val = evalPoly(P, t)
% Evaluate polynomial P(1) + P(2)*t + ... + P(n)*t^(n-1) via Horner.
    n = numel(P);
    val = P(n);
    for i = n-1:-1:1
        val = val * t + P(i);
    end
end

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end
