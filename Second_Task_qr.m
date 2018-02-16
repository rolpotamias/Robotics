function [crit,apost1,apost2]=Second_Task_qr(xd,yd,tk,centre_x,centre_y,diametre) 
%% Minimum Distance Critirion - Second Task Reference Velocity Calculus.
% Function Calculates Distance from Obstacles for each Joint and Arm.
% Sets Critirion V(q)=Minimum Distance and calculates reference velocity for each joint 
% via function 'Crit_derivative.m'. 
%
% Inputs :
% tk :  Current Time Frame
% i  : Current Joint 
% xd : Joint i X-axis Kinematic Position
% yd : Joint i X-axis Kinematic Position
% centre_x : Obstacles X-axis Centre Position
% centre_y : Obstacles Y-axis Centre Position
% diametre : Obstacles diametre

% Preallocate Distance Matrix to SpeedUp Calculations
    dist(1:8,1:2)=0; 
    x_mid(1:8)=0;
    y_mid(1:8)=0;
    Mid_dist(1:8,1:2)=0;
    
    dist(:,1)= sqrt((xd(tk,1:8)-centre_x).^2 + (yd(tk,1:8)-centre_y(1)).^2);
    dist(:,2)= sqrt((xd(tk,1:8)-centre_x).^2 + (yd(tk,1:8)-centre_y(2)).^2); 
    for i=1:8
        x_mid(i)=(xd(tk,i)+xd(tk,i+1))/2;
        y_mid(i)=(yd(tk,i)+yd(tk,i+1))/2;
        Mid_dist(i,1)=sqrt((x_mid(i)  -centre_x).^2 + (y_mid(i) -centre_y(1)).^2);
        Mid_dist(i,2)=sqrt((x_mid(i)  -centre_x).^2 + (y_mid(i) -centre_y(2)).^2);
    end
    %%%% Joint Minimum Distance %%%%%
    [Min_dist]=min(min(dist));
    [Ind,Obs]=find(dist==Min_dist);
    Ind=Ind(1);
    Obs=Obs(1);
    
    %%%% Arm Minimum Distance %%%%%
    [Min_mid_dist]=min(min(Mid_dist));
    [Ind_m,Obs_m]=find(Mid_dist==Min_mid_dist);
    Ind_m=Ind_m(1);
    Obs_m=Obs_m(1);
    apost1=[dist(:,1); Mid_dist(:,1)];
    apost2=[dist(:,2); Mid_dist(:,2)];
    
if (Min_mid_dist<2*diametre|| Min_dist<2*diametre)
    if (Min_mid_dist>Min_dist)
        if (Min_dist<diametre/2)
            fprintf('Warning! Joint %d Bumped on Obstacle No. %d \n',Ind,Obs);
        end
        [der_crit_x,der_crit_y]=Crit_derivative(tk,Ind,xd,yd); % Calculating Critirion Derivative
        Debug=[ones(Ind,1);zeros(9-Ind,1)];  % Keep only Valid Derivatives  
        crit=(1/Min_dist)*(( xd(tk,Ind)-centre_x )*der_crit_x + ( yd(tk,Ind)-centre_y(Obs) )*der_crit_y).*Debug;
    else
        if (Min_mid_dist<diametre/2)
            fprintf('Warning! Joint %d Bumped on Obstacle No. %d \n',Ind_m,Obs_m);
        end
        [der_crit_x1,der_crit_y1]=Crit_derivative(tk,Ind_m,xd,yd);   % Calculating Critirion Derivative
        [der_crit_x2,der_crit_y2]=Crit_derivative(tk,Ind_m+1,xd,yd); % Calculating Critirion Derivative
        der_crit_x=der_crit_x1+der_crit_x2;
        der_crit_y=der_crit_y1+der_crit_y2;
        Debug=[ones(Ind_m+1,1);zeros(8-Ind_m,1)]; % Keep only Valid Derivatives
        crit=(1/Min_mid_dist)*(( x_mid(Ind_m) -centre_x )*der_crit_x + ( y_mid(Ind_m) -centre_y(Obs_m) )*der_crit_y).*Debug;
    end
else
    crit=zeros(9,1);
end