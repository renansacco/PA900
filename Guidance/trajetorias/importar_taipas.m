% importar_taipas.m — Le shapefile de taipas e salva como trajetorias locais (x,y em metros)
%
% Gera taipas_boeck.mat com:
%   guias — cell array (Nx1), cada celula e struct com .x e .y [m]
%   origin — struct com .lat e .lon (ponto de referencia)

shpFile = fullfile(fileparts(mfilename('fullpath')), 'taipas-boeck', 'TAIPAS HELENA.shp');

%% Le shapefile (Mapping Toolbox)
S = shaperead(shpFile);
fprintf('Lidas %d polylines do shapefile\n', numel(S));

%% Converte lat/lon para ENU (metros) via geodetic2enu — WGS84 exato
% Referencia: primeiro ponto da primeira polyline
origin.lat = S(1).Y(1);
origin.lon = S(1).X(1);
wgs84 = wgs84Ellipsoid('meter');

guias = cell(numel(S), 1);
for i = 1:numel(S)
    lon = S(i).X(:);
    lat = S(i).Y(:);
    valid = ~isnan(lon) & ~isnan(lat);
    [xEast, yNorth, ~] = geodetic2enu(lat(valid), lon(valid), 0, ...
        origin.lat, origin.lon, 0, wgs84);
    guias{i}.x = xEast;
    guias{i}.y = yNorth;
end

%% Salva
outFile = fullfile(fileparts(mfilename('fullpath')), 'taipas_boeck.mat');
save(outFile, 'guias', 'origin');
fprintf('Salvo: taipas_boeck.mat (%d guias)\n', numel(guias));

%% Preview
figure('Name', 'Taipas Boeck');
hold on;
for i = 1:numel(guias)
    plot(guias{i}.x, guias{i}.y);
end
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('Taipas Boeck — %d guias', numel(guias)));
