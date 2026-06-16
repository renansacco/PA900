function U=controle(t,X)
global K_psi K_r psif 

psi=X(3); r=X(4); psid=psif;

upsi = (psid-psi)*K_psi + (0-r)*K_r;

if(upsi > 15)
    upsi = 15;
elseif (upsi<-15)
    upsi = -15;
end

vx=2.0;
U=[upsi; vx];