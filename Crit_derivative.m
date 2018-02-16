function [der_crit_x,der_crit_y]=Crit_derivative(tk,i,xd,yd)
%% Calculate Xd , Yd Partial Derivatives for Second Task
% [der_crit_x,der_crit_y]=Crit_derivative(tk,i,xd,yd)
% tk : Current Time Frame
% i  : Current Joint 
% xd : Joint i X-axis Kinematic Position
% yd : Joint i X-axis Kinematic Position

  der_crit_x=[ -yd(tk,i);
           -(yd(tk,i)-yd(tk,1));
           -(yd(tk,i)-yd(tk,2));
           -(yd(tk,i)-yd(tk,3));
           -(yd(tk,i)-yd(tk,4))*1.2;
           -(yd(tk,i)-yd(tk,5))*1.2;
           -(yd(tk,i)-yd(tk,6));
           -(yd(tk,i)-yd(tk,7));
           -(yd(tk,i)-yd(tk,8))];
  
       der_crit_y=[ +xd(tk,i);
          (xd(tk,i)-xd(tk,1));
          (xd(tk,i)-xd(tk,2));
          (xd(tk,i)-xd(tk,3));
          (xd(tk,i)-xd(tk,4));
          (xd(tk,i)-xd(tk,5));
          (xd(tk,i)-xd(tk,6));
          (xd(tk,i)-xd(tk,7));
          (xd(tk,i)-xd(tk,8))];

end