function [Jac,xd,yd]=Jacobian_Forward(qd,l,tk)
    s1=sin(qd(tk,1)+pi/2);
    c1=cos(qd(tk,1)+pi/2);
    
    s12=sin(qd(tk,1)+qd(tk,2));
    c12=cos(qd(tk,1)+qd(tk,2));
    
    s123=sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+pi/2);
    c123=cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+pi/2);
    
    s1234=sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+pi/2);
    c1234=cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+pi/2);
    
    s12345=sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5));
    c12345=cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5));

    s123456=sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)-pi/2);
    c123456=cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)-pi/2);
    
    s1234567=sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)-pi/2);
    c1234567=cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)-pi/2); 
    
    s12345678=sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)+qd(tk,8));
    c12345678=cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)+qd(tk,8)); 
    
    s123456789=sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)+qd(tk,8)+qd(tk,9));
    c123456789=cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)+qd(tk,8)+qd(tk,9));
    
    %%% JACOBIAN CALCULATION %%%%

    Jac(1,9)=-s123456789;

    Jac(1,8)= Jac(1,9)-s12345678;

    Jac(1,7)=Jac(1,8)- s1234567;

    Jac(1,6)=Jac(1,7)- s123456;

    Jac(1,5)= Jac(1,6) -s12345;

    Jac(1,4)=Jac(1,5)- s1234;

    Jac(1,3)=Jac(1,4)- s123;

    Jac(1,2)=Jac(1,3)- s12;

    Jac(1,1)=Jac(1,2)- s1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Jac(2,9)=c123456789;

    Jac(2,8)=Jac(2,9)+c12345678; 

    Jac(2,7)=Jac(2,8)+c1234567;

    Jac(2,6)=Jac(2,7)+c123456;

    Jac(2,5)=Jac(2,6)+c12345;

    Jac(2,4)=Jac(2,5)+c1234;

    Jac(2,3)=Jac(2,4)+c123;

    Jac(2,2)=Jac(2,3)+c12;

    Jac(2,1)= Jac(2,2)+c1;
    
   %% Forward Kinimatics - Joint Motion %%
   %%%%%% X-Cartesian Position of links %%
    xd(1)=l(1)*c1; 
    xd(2)=xd(1)+l(2)*c12;
    xd(3)=xd(2)+l(3)*c123;
    xd(4)=xd(3)+l(4)*c1234;
    xd(5)=xd(4)+l(5)*c12345;
    xd(6)=xd(5)+l(6)*c123456;
    xd(7)=xd(6)+l(7)*c1234567;
    xd(8)=xd(7)+l(8)*c12345678;
    xd(9)=xd(8)+l(9)*c123456789;
    %%%%%% Y-Cartesian Position of links %%
    yd(1)=l(1)*s1;
    yd(2)=yd(1)+l(2)*s12;
    yd(3)=yd(2)+l(3)*s123;
    yd(4)=yd(3)+l(4)*s1234;
    yd(5)=yd(4)+l(5)*s12345;
    yd(6)=yd(5)+l(6)*s123456;
    yd(7)=yd(6)+l(7)*s1234567;
    yd(8)=yd(7)+l(8)*s12345678;
    yd(9)=yd(8)+l(9)*s123456789;
    

end
