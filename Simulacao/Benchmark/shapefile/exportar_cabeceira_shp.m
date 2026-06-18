% exportar_cabeceira_shp.m — Exporta trajetoria cabeceira (reta+curva+reta) como shapefile
%
% Gera a mesma trajetoria do benchmark (gerar_trajetorias.m, traj #2)
% e salva como shapefile WGS-84 para uso no simulador embarcado.
%
% Requer: Mapping Toolbox (shapewrite, enu2geodetic)

%% Parametros da trajetoria
R = 6;                    % raio da curva [m]
len_reta = 30;            % comprimento de cada reta [m]
wpDist = 2.0;             % espacamento entre waypoints [m]

%% Origem geografica (ponto arbitrario — area plana no RS)
origin.lat = -29.0;
origin.lon = -53.0;

%% Gera trajetoria em coordenadas locais ENU (metros)
% Trecho 1: reta em +x
n1 = round(len_reta / wpDist);
x1 = linspace(0, len_reta, n1 + 1)';
y1 = zeros(size(x1));

% Trecho 2: semicirculo R=6m, centro em (len_reta, R)
n2 = round(pi * R / wpDist);
theta = linspace(0, pi, n2 + 1)';
theta = theta(2:end);
x2 = len_reta + R * sin(theta);
y2 = R * (1 - cos(theta));

% Trecho 3: reta em -x (volta)
n3 = round(len_reta / wpDist);
x3 = linspace(len_reta, 0, n3 + 1)';
x3 = x3(2:end);
y3 = 2 * R * ones(size(x3));

xLocal = [x1; x2; x3];
yLocal = [y1; y2; y3];

fprintf('Trajetoria: %d pontos, comprimento = %.1f m\n', ...
    numel(xLocal), sum(sqrt(diff(xLocal).^2 + diff(yLocal).^2)));

%% Converte ENU (metros) para WGS-84 (lat/lon)
wgs84 = wgs84Ellipsoid('meter');
[lat, lon, ~] = enu2geodetic(xLocal, yLocal, zeros(size(xLocal)), ...
    origin.lat, origin.lon, 0, wgs84);

%% Monta struct para shapewrite
S.Geometry = 'Line';
S.X = lon';
S.Y = lat';
S.ID = 1;
S.Nome = 'cabeceira_benchmark';
S.Length = sum(sqrt(diff(xLocal).^2 + diff(yLocal).^2));

%% Escreve shapefile
outDir = fileparts(mfilename('fullpath'));
outBase = fullfile(outDir, 'cabeceira_benchmark');
shapewrite(S, outBase);

%% Escreve .prj (WGS-84)
prjContent = 'GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137.0,298.257223563]],PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]]';
fid = fopen([outBase '.prj'], 'w');
fprintf(fid, '%s', prjContent);
fclose(fid);

fprintf('Shapefile salvo: %s (.shp/.shx/.dbf/.prj)\n', outBase);

%% Preview
figure('Name', 'Cabeceira — Shapefile');
subplot(1,2,1);
plot(xLocal, yLocal, '.-b');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Coordenadas locais (ENU)');

subplot(1,2,2);
plot(lon, lat, '.-r');
axis equal; grid on;
xlabel('Longitude [deg]'); ylabel('Latitude [deg]');
title('WGS-84 (lat/lon)');
sgtitle('Cabeceira benchmark — reta + curva R=6m + reta');
