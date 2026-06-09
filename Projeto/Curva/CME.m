function u = CME(y,yref,er,ur,B)
 
umin = ur(1); umax = ur(2);
% Cálculo do erro
et = yref - y; e = et / er;
if e < -1, e = -1;  elseif e > 1, e = 1; end
% Cálculo de u
if abs(e) == 1, ut = e; else ss = e - sign(e); ut = sign(ss) * (abs(ss) ^ (2 ^ -B) - 1); end
% Cálculo da saída
a = (umax - umin) / 2; u = a * (ut - 1) + umax;