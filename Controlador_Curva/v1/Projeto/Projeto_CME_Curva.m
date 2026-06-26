Ks = zeros(7,2);

% Ganho linear [psi r]
Ks(1,:) = [43.1006   52.7455];   % R=1/10^2 B=0
Ks(2,:) = [57.2296   61.8366];   % R=1/15^2 B=-1
Ks(3,:) = [69.7188   69.2583];   % R=1/20^2 B=-1
Ks(4,:) = [81.0910   75.6952];   % R=1/25^2 B=-1

Ks(5,:) = [57.2296   61.8366];   % R=1/15^2 B=1
Ks(6,:) = [69.7188   69.2583];   % R=1/20^2 B=1
Ks(7,:) = [81.0910   75.6952];   % R=1/25^2 B=1

ur_r = 15;
ur_psi=15;

B_array = [0,-1,-1,-1,1,1,1];



CME_Gains = zeros(7,8);
for i=1:7
    er_psi_array = (ur_psi/Ks(i, 1))*(2^-B_array(i));
    er_r_array = (ur_r/Ks(i, 2))*(2^-B_array(i));
    CME_Gains(i,:) = [er_psi_array, B_array(i), -ur_psi, ur_psi, er_r_array, B_array(i), -ur_r, ur_r];
end