function [ER] = RoboPercurso(Goal,K,P)
tic;
%definicao do Robo
RRobot = [0.5, 2.5, 3.5, 2.5, 0.5, 0, 0,0 
          -1, -1, 0, 1, 1, 0.5, 0,-0.5];
      
%definicao da origem do robo
 Xr= [0 0 25];

 Xo= Xr(1,1);
 Yo = Xr(1,2);
 Tho= Xr(1,3);
 ER=0;
 k=1;
for i=1:length(Goal(:,1))    
%definicao do objetivo do robo
   Xg = Goal(i,:);

%Gera o robo objetivo (Rotaciona e translada o robo referencia para o ponto objetivo)
GRobot = [cosd(Xg(3)) -sind(Xg(3))
          sind(Xg(3)) cosd(Xg(3))];
GRobot = GRobot*RRobot;
Tg=[1 0 Xg(1)
    0 1 Xg(2)
    0 0  1];
GRobot = [GRobot(1,:);GRobot(2,:);ones(1,8)];
GRobot = Tg*GRobot;

%coeficientes do controlador
KP = K(1);
KI =K(2);
KD = K(3);
vmax=500;

%dados iniciais
dt=0.01;
Dx = Xg(1)- Xr(1);
Dy = Xg(2) - Xr(2);
rho = sqrt(Dx^2+Dy^2);
tMax = 4;
%erro aceitavel 
episolon=5;

j=1;
%coeficientes da reta objetivo
% forma geral da reta: ax + by + c =0 , a=(y1-y2), b=(x2-x1) e c=(x1y2-x2y1)
a = Xr(1,2)-Goal(i,2);
b = Goal(i,1)-Xr(1,1);
c = Xr(1,1)*Goal(i,2)-Goal(i,1)*Xr(1,2);
while (abs(rho)>episolon)
    if toc >tMax
       break; 
    end
 %calculo dos dados do robo

    Dx = Xg(1)- Xr(1);
    Dy = Xg(2) - Xr(2);
    Dth = AjustaAngulo(Xg(3)-Xr(3));
    rho = sqrt(Dx^2+Dy^2);
    
    gamma = AjustaAngulo (rad2deg(atan2(Dy,Dx)));
    alpha(i,j) = AjustaAngulo (rad2deg(atan2(Dy,Dx))-Xr(3));
    
    
     %trás do robo
    if((gamma-Xr(3))>=-90 && (gamma-Xr(3))<=90)
        if(i<length(Goal))
           v=vmax;
        else
           v=min(KP*rho,vmax);
        end
        if j==1
           D= 0;
        else
            D=(alpha(i,j)-alpha(i,j-1));
        end
        w = min(KP*alpha(i,j)+KI*sum(alpha(i,:))+KD*D,360);  
    else
       if(i<length(Goal))
           v=-vmax;
        else
           v=-min(KP*rho,vmax);
       end
        
        if j==1
           D= 0;
        else
            D=(alpha(i,j)-alpha(i,j-1));
        end
        w = -min(KP*alpha(i,j)+KI*sum(alpha(i,:))+KD*D,360);  
    end  
    
    
    
    %Aplica no robo
    DS = v*dt;
    DTH = w*dt;
    
    X = [ DS*cosd(Xr(3)+DTH/2)
                DS*sind(Xr(3)+DTH/2)
                DTH];
    Xr =Xr+X';     
    %erro:    
    ER(i,j) = abs(a*Xr(1,1)+b*Xr(1,2)+c)/sqrt((a^2)+(b^2));
    
   %Gera o robo no novo ponto (Rotaciona e translada o robo para o novo ponto)
   Robot = [cosd(Xr(3)) -sind(Xr(3))
          sind(Xr(3)) cosd(Xr(3))];
   Robot = Robot*RRobot;
    Tg=[1 0 Xr(1)
        0 1 Xr(2)
        0 0  1];
    Robot = [Robot(1,:);Robot(2,:);ones(1,8)];
    Robot = Tg*Robot;
    
    Path(k,:)=[Robot(1,7) Robot(2,7)];
    k=k+1;
    j=j+1;
end

 %Plot
    if P==1
        close all;
        figure
        hold off;
        subplot(1,2,1);
        fill(Robot(1,:) , Robot(2,:) , 'r');
        hold on;
        plot([Xo Goal(i,1)],[ Yo Goal(i,2)]);
        scatter(GRobot(1,7) , GRobot(2,7) ,50, 'g');
        plot(Path(:,1) , Path(:,2) , 'r');
        title('Percurso do Robo');
        axis equal; grid on; xlabel('x'), ylabel('y');
        
        subplot(1,2,2);
        plot(ER(i,:));
        title(['K=[ ' num2str(K(1)) ', ' num2str(K(2)) ',' num2str(K(3)) '] - Erro: '  num2str(sum(ER))]);
        grid on; xlabel('t'), ylabel('Erro');
        drawnow;
    end


end
ER=sum(ER);
end