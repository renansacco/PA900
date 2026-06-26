function f=objetivo(Ks)
global X0 K_psi tsim psif ts K_r

K_psi=Ks(1); K_r=Ks(2);
t=[]; X=[]; U=[];
[t,X]=ode45(@dinamica_MF,[0 tsim],X0);
for i=1:length(t)
    Ui=controle(t(i),X(i,:)');
    U=[U; Ui'];
end

% for i=1:length(t)
%     rc(i)=psif;
%     if ts > 0 && t(i) < ts
%         rc(i)=psif*(2*t(i)/ts-(t(i)/ts)^2);
%     end     
% end
% f=sum(t.*abs((rc'-X(:,3))/psif));

for i=1:length(t)
    rc(i)=psif;
end

R = 1/25^2;
Q_psi = 1/deg2rad(10)^2;
Q_r = 0;
f = trapz(t, t.*(Q_psi*(rc'-X(:,3)).^2 + Q_r*X(:,4).^2)) + trapz(t, R*U(:,1)'.^2);

% f = trapz(t, (((rc'-X(:,3))/psif).^2));

figure(1);
subplot(221); plot(t,X(:,3)*180/pi,t,rc*180/pi,'r'); xlabel('t [s]'); ylabel('\psi [°]');
subplot(222); plot(t,U(:,1)); xlabel('t [s]'); ylabel('\omega_{m,ref} [°]');
subplot(223); plot(t,X(:,6)*180/pi); xlabel('t [s]'); ylabel('\delta [°]');
subplot(224); plot(t,X(:,4)*180/pi); xlabel('t [s]'); ylabel('r [°/s]');
drawnow; 

end