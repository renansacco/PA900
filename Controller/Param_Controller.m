%% Paths
ctrlDir = fileparts(mfilename('fullpath'));
addpath(fullfile(ctrlDir, 'buses'));
addpath(fullfile(ctrlDir, 'enums'));
addpath(fullfile(ctrlDir, 'gains'));
addpath(fullfile(pwd, 'Controller_ert_rtw'));
%% Buses
Simulink.importExternalCTypes(fullfile(ctrlDir, 'Controller_ert_rtw', 'mg900_model_types.h'));

Bus_Controlador_UserInput
Bus_Controlador

controllerParameterBus
controllerMeasurementBus
controllerSteerCalibrationBus
controllerInputGuidanceBus

%% Parametros de calibra��o da dire��o
steerCalibration = Simulink.Parameter;
steerCalibration.DataType = 'Bus: controllerSteerCalibrationBus_t';
steerCalibration.Value = struct('leftValue',0,'rightValue',0,'centerValue', 0,'isCalibrated',false, 'deadZone', 0);
steerCalibration.CoderInfo.StorageClass = 'Model default';

%% Parametros do usu�rio
userParameters = Simulink.Parameter;
userParameters.DataType = 'Bus: controllerParameterBus_t';
userParameters.Value = struct('modelPlant', 1, 'vehicleMode', ModoVeiculo.None, 'straightAggressiveness', 4, 'curveAggressiveness', 1, 'isCurveMode', false);
userParameters.CoderInfo.StorageClass = 'Model default';


%% Controlador de curva
Controlador = Simulink.Parameter;
Controlador.Value.Ts = 0.05;

param_curva = load('Curva_Gains_Linear.mat');

Controlador.Value.Curva.Gains = single(param_curva.Gains_Curva);
Controlador.Value.Curva.v_index = single(param_curva.vx_table);
Controlador.Value.Curva.T_look = single(param_curva.T_look);
Controlador.Value.Curva.omegam_sat = single(6);

%% Controlador de curva — CME legado (teste comparativo)
% Ks_cme = [0.1817   1    0.1089   1];
% Controlador.Value.CurvaCME.eps = Ks_cme(1);
% Controlador.Value.CurvaCME.Bps = Ks_cme(2);
% Controlador.Value.CurvaCME.urps = single([-15 15]');
% Controlador.Value.CurvaCME.er = Ks_cme(3);
% Controlador.Value.CurvaCME.Br = Ks_cme(4);
% Controlador.Value.CurvaCME.urr = single([-15 15]');
% param_cme = load('Curva_Gains_Final.mat');
% controllerGainsCurve = param_cme.CME_Gains;
% Controlador.Value.CurvaCME.omegam_sat = single(15);
% Controlador.Value.CurvaCME.Delta = single(4);

%% Controlador 'Keep'
param_keep_trator = load('Keep_Tractor_Sem_Implemento.mat');
param_keep_sulcon = load('Keep_Tractor_Implemento_Leve.mat');
param_keep_sulcon_3 = load('Keep_Tractor_Implemento_Medio.mat');
param_keep_sulcon_5 = load('Keep_Tractor_Implemento_Pesado.mat');

Gains = cat(4, param_keep_trator.Gains_Keep_Tractor, param_keep_sulcon.Gains_Keep_Tractor, param_keep_sulcon_3.Gains_Keep_Tractor, param_keep_sulcon_5.Gains_Keep_Tractor);

Controlador.Value.Keep.Gains = single(Gains);
Controlador.Value.Keep.R_index = single(param_keep_trator.R_table);
Controlador.Value.Keep.v_index = single(param_keep_trator.vx_table);
Controlador.Value.Keep.omegam_sat = 5;

%% Malha interna (atuador)
k_lqr_MI = [0.2914   38.1972]; %X=[omegam,delta]

%% Controlador Entry
param_entry_trator = load('Entry_Tractor.mat');

Controlador.Value.Entry.omegam_sat = 10;
Controlador.Value.Entry.Delta = single(6);

Gains_Entry = cat(4, param_entry_trator.Gains_Keep_Tractor, param_entry_trator.Gains_Keep_Tractor); %%%%%%% nao tem o controle com implemento

Controlador.Value.Entry.Gains = single(Gains_Entry);
Controlador.Value.Entry.R_index = single(param_entry_trator.R_table);
Controlador.Value.Entry.v_index = single(param_entry_trator.vx_table);
