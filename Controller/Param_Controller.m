%% Buses
Simulink.importExternalCTypes('mg900_model_types.h');

Bus_Controlador_UserInput
Bus_Controlador

controllerParameterBus
controllerMeasurementBus
controllerSteerCalibrationBus
controllerInputGuidanceBus

%% Parametros de calibração da direção
steerCalibration = Simulink.Parameter;
steerCalibration.DataType = 'Bus: controllerSteerCalibrationBus_t';
steerCalibration.Value = struct('leftValue',0,'rightValue',0,'centerValue', 0,'isCalibrated',false, 'deadZone', 0);
steerCalibration.CoderInfo.StorageClass = 'Model default';

%% Parametros do usuário
userParameters = Simulink.Parameter;
userParameters.DataType = 'Bus: controllerParameterBus_t';
userParameters.Value = struct('modelPlant', 1, 'vehicleMode', ModoVeiculo.None, 'straightAggressiveness', 4, 'curveAggressiveness', 4, 'isCurveMode', false);
userParameters.CoderInfo.StorageClass = 'Model default';


%% Controlador de curva
Controlador = Simulink.Parameter;
Controlador.Value.Ts = 0.05;


%Ks=[8.7696   -6.7563    0.1573   -0.8825]; % original, rodrigo

Ks = [0.1817   1    0.1089   1];      % Projeto baseado nos ganhos lineares [K_psi, K_r]=[41.28, 68.9].  B=1 

% CME de 'psi'
Controlador.Value.Curva.eps = Ks(1);
Controlador.Value.Curva.Bps = Ks(2);
Controlador.Value.Curva.urps = single([-15 15]');

% CME de 'r'
Controlador.Value.Curva.er = Ks(3);
Controlador.Value.Curva.Br = Ks(4);
Controlador.Value.Curva.urr = single([-15 15]');

param_cme = load('Curva_Gains_Final.mat');
%Controlador.Value.Curva.Gains = Simulink.Parameter;
controllerGainsCurve = param_cme.CME_Gains;

% Saturação da velocidade angular de referência
Controlador.Value.Curva.omegam_sat = single(15);

% Distância 'Delta' do lookahead
Controlador.Value.Curva.Delta = single(4);

%% Controlador 'Keep'
param_keep_trator = load('gains/Keep_Tractor_Sem_Implemento.mat');
param_keep_sulcon = load('gains/Keep_Tractor_Implemento_Leve.mat');
param_keep_sulcon_3 = load('gains/Keep_Tractor_Implemento_Medio.mat');
param_keep_sulcon_5 = load('gains/Keep_Tractor_Implemento_Pesado.mat');

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
