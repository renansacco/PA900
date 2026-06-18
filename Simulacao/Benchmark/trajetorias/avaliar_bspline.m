function [sx, sy] = avaliar_bspline(wps, npts_per_seg)
% AVALIAR_BSPLINE  Avalia o B-spline cubico uniforme do guidance
%
% Usa a mesma logica de computeSpline do guidance_step.m para gerar
% a curva que o guidance efetivamente produz a partir dos waypoints.
%
%   [sx, sy] = avaliar_bspline(wps)
%   [sx, sy] = avaliar_bspline(wps, 50)  % 50 pontos por segmento
%
% Input:
%   wps — struct com .x e .y (waypoints de entrada do guidance)
%   npts_per_seg — pontos por segmento (default: 20)
%
% Output:
%   sx, sy — coordenadas da curva B-spline avaliada

if nargin < 2, npts_per_seg = 20; end

wx = wps.x(:);
wy = wps.y(:);
n = numel(wx);

sx = zeros((n-1) * npts_per_seg, 1);
sy = zeros((n-1) * npts_per_seg, 1);
k = 0;

t_vec = linspace(0, 1, npts_per_seg + 1);
t_vec = t_vec(1:end-1);  % remove t=1 (coincide com t=0 do proximo segmento)

for wpIndex = 2:n
    [xc, yc] = computeSplineLocal(wx, wy, n, wpIndex);

    if wpIndex == n
        t_eval = linspace(0, 1, npts_per_seg);
    else
        t_eval = t_vec;
    end

    for j = 1:numel(t_eval)
        t = t_eval(j);
        k = k + 1;
        sx(k) = xc(1) + xc(2)*t + xc(3)*t^2 + xc(4)*t^3;
        sy(k) = yc(1) + yc(2)*t + yc(3)*t^2 + yc(4)*t^3;
    end
end

sx = sx(1:k);
sy = sy(1:k);
end

function [xc, yc] = computeSplineLocal(wx, wy, n, wpIndex)
% Replica exata de computeSpline do guidance_step.m
% B-spline cubico uniforme com phantom point mirroring nas bordas

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

    cx_vec = M' * Px;
    cy_vec = M' * Py;

    xc = cx_vec';
    yc = cy_vec';
end
