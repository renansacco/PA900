function wps_out = resample_waypoints(wps_in, wpDistance)
% RESAMPLE_WAYPOINTS  Reamostra waypoints a espacamento uniforme
%
% Replica a logica de interpolateWaypoints.m do ERT/embarcado:
% interpolacao linear ao longo do arc-length, espacamento uniforme.
%
%   wps_out = resample_waypoints(wps_in, 3.0)
%
% Input:
%   wps_in     — struct com .x e .y (Nx1)
%   wpDistance  — espacamento desejado [m]
%
% Output:
%   wps_out    — struct com .x e .y reamostrados

x = wps_in.x(:);
y = wps_in.y(:);
n = numel(x);

d = sqrt(diff(x).^2 + diff(y).^2);
s = [0; cumsum(d)];
totalLen = s(end);

N_points = ceil(totalLen / wpDistance);
if N_points < 2
    wps_out = wps_in;
    return;
end

s_uniform = linspace(0, totalLen, N_points)';

wps_out.x = interp1(s, x, s_uniform, 'linear');
wps_out.y = interp1(s, y, s_uniform, 'linear');

end
