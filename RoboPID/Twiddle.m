close all;clear all;clc;
 
tic;
%inicializa
K = [0 0 0];
dK = [1 1 1];
d = 0.1;
ksi = 0.1;
Goal= [-1000 -1000 0];

P=0;
Ebest = RoboPercurso(Goal,K,P);
 
while sum(dK) > d
    for i = 1:length(K)
        
        K(i) = K(i) + dK(i);
        E = RoboPercurso(Goal,K,P);
        
        if E < Ebest
            Ebest = E;
            dK(i) = dK(i) * (ksi + 1);
        else
             K(i) = K(i) - 2 * dK(i);
             E = RoboPercurso(Goal,K,P);
             if E < Ebest
                 Ebest = E;
                 dK(i) = dK(i) * (ksi + 1);
             else
                 K(i) = K(i) + dK(i);
                 dK(i) = dK(i) * (1 - ksi);
             end
        end
    end
end 
E = RoboPercurso(Goal,K,1);

