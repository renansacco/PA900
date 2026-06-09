% ur: sinais de controle máximos tolerados
% er: sinal de erro máximo tolerado
% B: coeficiente da concavidade
function dfdy = LinearizaCME(er, ur, B, iplot)

% Plota curva entrada/saída do CME
e = linspace(-er, er, 1000);        % sinal de erro
u = CME_array(e, er, ur,B);  % sinal de controle

% Lineariza o CME 
dy = er/1e4;
dfdy = (CME(dy,er,ur,B)-CME(-dy,er,ur,B))/(2*dy);

if iplot==1
    % Plota valores
    figure(1)
    hold on
    plot(e, u)
    hold on
    plot(e, e*dfdy)
    legend('CME', 'CME Linearizado')
end
end


function u = CME_array(et,er,ur,Br)
u=zeros(1,size(et,2));
for i=1:size(et,2)
    u(i) = CME(et(i),er,ur,Br);
end
end

function u = CME(et,er,ur,B)
 
umin = ur(1); umax = ur(2);
% Cálculo do erro
%et = yref - y;
e = et / er;
if e < -1, e = -1;  elseif e > 1, e = 1; end
% Cálculo de u
if abs(e) == 1, ut = e; else ss = e - sign(e); ut = sign(ss) * (abs(ss) ^ (2 ^ -B) - 1); end
% Cálculo da saída
a = (umax - umin) / 2; u = a * (ut - 1) + umax;

end