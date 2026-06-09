function f=objetivo(Ks, iplot,tau)
global X0 K_psi tsim K_r K_e
global Q R P Pr

K_psi=Ks(1); K_r=Ks(2); K_e = Ks(3);
t=[]; X=[]; U=[];
[t,X]=ode45(@dinamica_MF,[0 tsim],X0);
for i=1:length(t)
    Ui=controle(t(i),X(i,:)');
    U=[U; Ui'];
end



%% Função de custo quadrática c/ integral do tempo
% Je = zeros(size(t,2),1);
% Ju = zeros(size(t,2),1);
% for i=1:length(t)
%     Ji(i) = X(i,:)*Q*X(i,:)' + U(i,1)*R*U(i,1)' + t(i)*X(i,:)*P*X(i,:)' + t(i)*U(i,1)*Pr*U(i,1)';
% end
% f = trapz(t, Ji);

yref = 0.1*exp(-t/tau);
for i=1:length(t)
    Xe = [0,0,0,0,0,0,0,yref(i)]-X(i,:);
    %Ji(i) = Xe*Q*Xe' + U(i,1)*R*U(i,1)' + t(i)*Xe*P*Xe' + t(i)*U(i,1)*Pr*U(i,1)';
    
    Ji(i) = t(i)*(dot(abs(Xe), diag(P)) + Pr*abs(U(i,1))) + R*abs(U(i,1)) + dot(abs(Xe), diag(Q));
%     Je(i) = dot(abs(Xe), diag(Q));
%     Ju(i) = R*abs(U(i,1));
end
f = trapz(t, Ji);

% f = trapz(t, (Q_e*X(:,8).^2)) + trapz(t, (R*U(:,1).^2)) + trapz(t, Q_r*X(:,4).^2);
% f = trapz(t, t.*abs(X(:,8))) + trapz(t, (R*U(:,1).^2));

%% TESTE PENALIZAÇÃO AUTOVALOR IMAGINÁRIO
% global Cf Cr Ch Lf Lr Lh Lcp Izz m tau km kd
% [A, B] = linearizar(@(xe,ue) dinamica(xe,[ue, 1.5], Cf,Cr,Ch,Lf,Lr,Lh,Lcp,Izz,m,tau,km,kd), zeros(8,1), 0);
% C = [1,0,0,0,0,0;
%     0,1,0,0,0,0;
%     0,0,0,0,0,1];
% Ac = A(3:end, 3:end)-B(3:end)*Ks*C;
% 
% eigAc = eig(Ac);
% [eig_sort_Re, isort] = sort(abs(real(eigAc)));
% eig_sort_Im = sort(abs(imag(eigAc)));
% 
% f = f*exp(5*abs(imag(eigAc(isort(1)))));       %Polo mais significativo sem parte imaginaria (resposta de primeira ordem)
% % f=f*exp(5*eig_sort_Im(6))
% if(eig_sort_Re(1)/eig_sort_Re(2) < 2)
%     f = f*exp(2*eig_sort_Re(1)/eig_sort_Re(2));
% end
% f=f*exp(5*abs(max(imag(eigAc))));
% 
% 
% eig_sort = sort(abs(real(eigAc)));
% 
% f=f*exp(3*abs(eig_sort(1)-0.4))*exp(3*abs(eig_sort(2)-1));

%%
if(iplot)
    figure(1);
    subplot(321); plot(t,X(:,3)*180/pi); xlabel('t [s]'); ylabel('\psi [°]');
    subplot(322); plot(t,U(:,1)); xlabel('t [s]'); ylabel('\omega_{m,ref} [°]');
    subplot(323); plot(t,X(:,6)*180/pi); xlabel('t [s]'); ylabel('\delta [°]');
    subplot(324); plot(t,X(:,4)*180/pi); xlabel('t [s]'); ylabel('r [°/s]');
    subplot(325); plot(t,X(:,8),t,yref); xlabel('t [s]'); ylabel('e [m]');
    subplot(326); plot(t,X(:,5)); xlabel('t [s]'); ylabel('v_y [m/s]');
    drawnow; 
end

end