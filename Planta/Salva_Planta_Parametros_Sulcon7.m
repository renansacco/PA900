%% Parametros para Simulação Veiculo + implemento
clear
i_usar_implemento = 1;

%%
m = 7500;             %(kg)
Izz = 10000;          %(kg*m^2)

% Dimensões
Lr = 0.75;            %(m)
Lf = 1.75;            %(m)
La = 1.80;

% Rodas frontais
% C1 = 1000*180/pi;     %(N/rad)
% C2 = 1000*180/pi;     %(N/rad)
% Cf = C1+C2;
Cf = 1000*180/pi;

% Rodas traseiras
% C3 = 2000*180/pi;     %(N/rad)
% C4 = 2000*180/pi;     %(N/rad)
% Cr = C3+C4;
Cr = 5000*180/pi;

% Ganhos do atuador
k_d = 0.0437;
k_m = 1; %Não há engrenagem quando se utiliza o servomotor
tau = 0.2;

%% Implemento Sulcon-5
mh = 3500;            % Massa do implemento (kg)
Lh = 1.5;             % Distância do implemento até a roda traseita (metros)
Ch = 1400*180/pi;     % Coeficiente de rigidez lateral do implemento (N/rad)

if i_usar_implemento==1
    [m, Izz, Lr, Lf] = Parametros_Veiculo_Implemento(Lr, Lf, Lh, m, mh);
else
    mh=0;
    Lh=0;
    Ch=0;
end

% Distância do ponto de controle situado no implemento
%L_cp = -(Lr+Lh);
% L_cp = -(Lr);
L_cp = 0;

% name = input('Nome do arquivo para salvar: ', 's');
% if(~isempty(name))
%     save(name)
% end
save('param_MF6713_Sulcon7.mat')

%% Cálcula os parâmetros da planta com implemento
% Recebe os parametros 'Lr', 'Lf' e 'm' do veículo (sem implemento), junto
% com os parâmetros 'mh' e 'Lh' do implemento. Estima os
% parâmetros atualizados da planta, considerando a massa localizada nas
% rodas.
function [m_i, Izz_i, Lr_i, Lf_i] = Parametros_Veiculo_Implemento(Lr, Lf, Lh, m, mh)

mf = m*Lr/(Lr+Lf); % massa na roda frontal
mr = m*Lf/(Lr+Lf); % massa na roda frontal


c = (mf*(Lr+Lf)-mh*Lh)/(mf+mr+mh); % distância do centro de massa até a roda frontal. Positiva (a frente da roda), Negativa (atrás da roda)

Izz_i = (Lf+Lr-c)^2*mf + c^2*mr + (c+Lh)^2*mh;
m_i = m + mh;
Lr_i = c;
Lf_i = (Lr+Lf-c);
end