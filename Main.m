%%%%%%%%%%%%%%%%%% ROBOTICS II - 8th Semester %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Potamias Rolandos - Alexandros  %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%     AM 03114437       %%%%%%%%%%%%%%%%%%%%%
clear all ;
close all ;
clc;
tic;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initial Configuration %%
%%% Start Point %%%
xd_start=4;
yd_start=1;
%%% Joint Lengths %%%
l(1:9)=1;
%%%% Sampling Period %%%%  
dt=0.001;
plot_period=50;
%%%% Motion Duration%%%%%
Tf=1; 
%%%% Time Array %%%
t=0:dt:Tf ;
Num=Tf/dt+1; % Loop Number

%%%% Joints %%%%%
%%%% Inititan Joints Configuration %%%%
q_0(1:9)=0;
%%%%% Preallocate Total Configuration
qd(1:Num,1:9)=0; 
q_dot(1:Num,1:9)=0;
xd(1:Num,1:9)=0;
yd(1:Num,1:9)=0;
%%%% Obstacle %%%%
centre_x=4;
centre_y(1)=1+0.75;
centre_y(2)=1-0.75;
x_obs(1:2)=centre_x-0.25;
y_obs(1)=centre_y(1)-0.25;
y_obs(2)=centre_y(2)-0.25;
diametre=0.5;
%% FINAL POSITION %%
xd_final=xd_start+3;
yd_final=yd_start;

%% FIRST TASK - DESIRED TRAJECTORY %%%
tk=1;
qd(tk,1)=q_0(1); qd(tk,2)=q_0(2); qd(tk,3)=q_0(3); 
qd(tk,4)=q_0(4); qd(tk,5)=q_0(5); qd(tk,6)=q_0(6); 
qd(tk,7)=q_0(7); qd(tk,8)=q_0(8); qd(tk,9)=q_0(9);
%%%% Desired Trajectory %%%%%
pd_x=zeros(Num); % Trajectory Matrix Preallocation
pd_y=zeros(Num);
lamda_x=repmat((xd_final-xd_start)/Tf,Num);  % X-axis Angular Velocity
lamda_y=repmat((yd_final-yd_start)/Tf,Num);  % Y-axis Angular Velocity
pd_x(1)=xd_start;
pd_y(1)=yd_start;
for i=2:Num
    pd_x(i)=pd_x(i-1)+lamda_x(i)*dt;
    pd_y(i)=pd_y(i-1)+lamda_y(i)*dt;
end
%% Uncommend to add increased Velocity to Desired Trajectory(Polyonimial Interpolation)
%[pd_x,lamda_x]=Interpolation(xd_start,xd_final,Tf,Num);
%[pd_y,lamda_y]=Interpolation(yd_start,yd_final,Tf,Num);
%% Starting Kinematic Simulation %%
%Id_matrix=[1;zeros(8,1)]; %Co-Multipling Second Task 
time=0;
Bold=0;
Color='r';
Stop=0;
%% GUInterface Control
f0=figure;
h1=uicontrol('Style','pushbutton','String','Up',   'Position',[20 150 35 20],'Callback','centre_y(1)=centre_y(1)+0.1; centre_y(2)=centre_y(2)+0.1;') ;
h2=uicontrol('Style','pushbutton','String','Down', 'Position',[20 110 35 20],'Callback','centre_y(1)=centre_y(1)-0.1; centre_y(2)=centre_y(2)-0.1;') ;
h3=uicontrol('Style','pushbutton','String','Left', 'Position',[0  130 35 20],'Callback','centre_x=centre_x-0.1;') ;
h4=uicontrol('Style','pushbutton','String','Right','Position',[40 130 35 20],'Callback','centre_x=centre_x+0.1;') ;
h5=uicontrol('Style','pushbutton','String','Stop','Position', [20 250 35 20],'Callback','Stop=1;') ;
while time<=Tf 
    %% Obstacle GUI Debugging
    if (Stop==1)
        display('Quiting Kinimatic Simulation....');
        break;
    end
    x_obs(1:2)=centre_x-0.25;
    y_obs(1)=centre_y(1)-0.25;
    y_obs(2)=centre_y(2)-0.25;
    if(y_obs(1)>2||y_obs(2)<-0.5)
        display('Error! Obstacle reached Y-axis Limit');
        display('Reseting Position.....');
        centre_y(1)=1.75;
        centre_y(2)=0.25;
    end
    if (abs(x_obs-3.75)>0.5)
        display('Error! Obstacle reached X-axis Limit');
        display('Reseting Position.....');
        centre_x=4;
    end
        
   %% Jacobian and Forward Kinimatics Calculation %%
    [Jac,xd(tk,:),yd(tk,:)]=Jacobian_Forward(qd,l,tk);
    if (tk==1)
        StickDiagram(xd,yd,pd_x,pd_y,x_obs,y_obs,diametre,tk,1,'b') ;
        pause(0.5);
    end
   %% Pseudo-Inverse Jacobian Matrix %%
    Jac_pinv = Jac'*inv(Jac*Jac');  
   %% - Subtask No. 1 - %%
    task1=Jac_pinv*[lamda_x(tk);lamda_y(tk)];
   %% - Subtask No. 2 - %%
    Kc=5.5*eye(9);
    [crit,apost1(tk,:),apost2(tk,:)]=Second_Task_qr(xd,yd,tk,centre_x,centre_y,diametre); 
    task2=(eye(9)-Jac_pinv*Jac)*Kc*crit;

   %% Kinematic Simulation - Joint Calculation(Integration) %%
    %%%% Angular Velocity %%%%
    q_dot(tk,:)=task1'+task2';
     %%% Next Position %%%
    qd(tk+1,1)=qd(tk,1)+dt*q_dot(tk,1);
    qd(tk+1,2)=qd(tk,2)+dt*q_dot(tk,2);
    qd(tk+1,3)=qd(tk,3)+dt*q_dot(tk,3);
    qd(tk+1,4)=qd(tk,4)+dt*q_dot(tk,4);
    qd(tk+1,5)=qd(tk,5)+dt*q_dot(tk,5);
    qd(tk+1,6)=qd(tk,6)+dt*q_dot(tk,6);
    qd(tk+1,7)=qd(tk,7)+dt*q_dot(tk,7);
    qd(tk+1,8)=qd(tk,8)+dt*q_dot(tk,8);
    qd(tk+1,9)=qd(tk,9)+dt*q_dot(tk,9);

   %% Ploting %%
   if (tk==1000)
       Bold=1;
       Color='k';
   end
   if (mod(tk,plot_period)==0)  % Plot 
        StickDiagram(xd,yd,pd_x,pd_y,x_obs,y_obs,diametre,tk,Bold,Color);
        pause(0.4);
   end 
   tk=tk+1;
   time=time+dt;  
end
%% Plot 
%%%% Ploting Angle of Joins and their Distances from Obstacles 
Plot_results(qd,apost1,apost2);
