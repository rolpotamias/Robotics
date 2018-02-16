function [py,lamda]=Interpolation(yd0,yd1,Tf,Points)
        %%%% 3rd order Polyonimal Interpolation %%%
        %%%% After Solving the parametres of a3*(t^3)+a2*(t^2) + a1*(t)+a0 %%%%
    dt=0.001;    %Time presision 1msec
    t=0:dt:Tf;   %Time Scale
    a=(yd1-yd0)/0.8;
    for i =1:Points 
        if i<=(Points/5)
        py(i)=(a/0.4)*t(i)^2+yd0;
        elseif i<(0.8*Points)
        py(i)=a*t(i)+(yd0-0.1*a);
        else
        py(i)=-(a/0.4)*t(i)^2+(a/0.2)*t(i)+(yd1-2.5*a);
        end
    end
   
    lamda=diff(py)/dt; % Velocity
    
        
end
