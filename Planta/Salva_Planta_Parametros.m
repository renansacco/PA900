%% Modelo Linearizado (Erro Lateral + Dinamica Lateral + Atuador SERVOMOTOR)

%% Parametros para Simulação Veiculos em MA
clear

m = 7500;             %(kg)
Izz = 10000;          %(kg*m^2)

% Dimensões
Lr = 0.75;            %(m)
Lf = 1.75;            %(m)
La = 1.80;

% Distância do ponto de controle
L_cp = 0;

% Rodas frontais
C1 = 2000*180/pi;     %(N/rad)
C2 = 2000*180/pi;     %(N/rad)
Cf = C1+C2;

% Rodas traseiras
C3 = 4000*180/pi;     %(N/rad)
C4 = 4000*180/pi;     %(N/rad)
Cr = C3+C4;


% Ganhos do atuador
k_d = 0.0437;
k_m = 1; %Não há engrenagem quando se utiliza o servomotor
tau = 0.2;

name = input('Nome do arquivo para salvar: ', 's');
if(~isempty(name))
    save(name)
end