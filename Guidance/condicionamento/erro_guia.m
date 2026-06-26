function erros = erro_guia(wps_original, guia)
% ERRO_GUIA  Distancia de cada waypoint original ate a guia interpolada.
%
%   erros = erro_guia(wps_original, guia)
%
%   Input:
%       wps_original — struct com .x e .y (waypoints brutos)
%       guia         — struct retornada por condicionar_guia
%
%   Output:
%       erros — (N x 1) distancia minima de cada wp ate a polyline (guia.x, guia.y)

    wx = wps_original.x(:);
    wy = wps_original.y(:);
    gx = guia.x;
    gy = guia.y;
    n_wp = numel(wx);

    erros = zeros(n_wp, 1);
    for i = 1:n_wp
        erros(i) = dist_ponto_polyline(wx(i), wy(i), gx, gy);
    end
end

function d = dist_ponto_polyline(px, py, gx, gy)
    n = numel(gx);
    d = inf;
    for i = 1:n-1
        abx = gx(i+1) - gx(i);
        aby = gy(i+1) - gy(i);
        apx = px - gx(i);
        apy = py - gy(i);
        len2 = abx^2 + aby^2;
        if len2 < 1e-12
            di = sqrt(apx^2 + apy^2);
        else
            t = max(0, min(1, (apx*abx + apy*aby) / len2));
            cx = gx(i) + t*abx;
            cy = gy(i) + t*aby;
            di = sqrt((px - cx)^2 + (py - cy)^2);
        end
        if di < d, d = di; end
    end
end
