function StickDiagram(xd,yd,pd_x,pd_y,x_obs,y_obs,diametre,tk,Bold,Color)
%% Diplay Kinimatic Simulation with Stick Diagram Format
% Inputs 
% tk :  Current Time Frame
% i  : Current Joint 
% xd : Joint i X-axis Kinematic Position
% yd : Joint i X-axis Kinematic Position
% pd_x : Desired X-axis Position
% pd_y : Desired Y-axis Position
% x_obs: X-axis obstacle Position
% y_obs: Y-axis obstacle Position
% diametre: Obstacle Diametre
% Bold  : Stick-Lines Width
% Color : Stick-Lines Color
% To get all the time frames at the same plot COMMEND cla(line 16).
        cla;
        axis([-1 8 -1 3.5]);
        xlabel('X-axis');
        ylabel('Y-axis');
        %%%% Trajectory %%%%
        plot(pd_x,pd_y,'-m');
        hold on;
        %%%%% Obstacle %%%%%
        posUp   = [x_obs(1) y_obs(1) diametre diametre]; 
        posDown = [x_obs(2) y_obs(2) diametre diametre]; 
        rectangle('Position',posUp,'Curvature',[1 1],'FaceColor','k','EdgeColor','r');
        rectangle('Position',posDown,'Curvature',[1 1],'FaceColor','k','EdgeColor','r');
       
        plot([0, xd(tk,1)], [0, yd(tk,1)],Color,'LineWidth',2*Bold+0.5);
        plot([0], [0], 'o');
 
        plot([xd(tk,1), xd(tk,2)], [yd(tk,1), yd(tk,2)],Color,'LineWidth',2*Bold +0.5);
        plot([xd(tk,1)], [yd(tk,1)], 'o');
 
        plot([xd(tk,2), xd(tk,3)], [yd(tk,2), yd(tk,3)],Color,'LineWidth',2*Bold+0.5);
        plot([xd(tk,2)], [yd(tk,2)], 'o');
 
        plot([xd(tk,3), xd(tk,4)], [yd(tk,3), yd(tk,4)],Color,'LineWidth',2*Bold+0.5);
        plot([xd(tk,3)], [yd(tk,3)], 'o');
 
        plot([xd(tk,4), xd(tk,5)], [yd(tk,4), yd(tk,5)],Color,'LineWidth',2*Bold+0.5);
        plot([xd(tk,4)], [yd(tk,4)], 'o');
 
        plot([xd(tk,5), xd(tk,6)], [yd(tk,5), yd(tk,6)],Color,'LineWidth',2*Bold+0.5);
        plot([xd(tk,5)], [yd(tk,5)], 'o');
 
        plot([xd(tk,6), xd(tk,7)], [yd(tk,6), yd(tk,7)],Color,'LineWidth',2*Bold+0.5);
        plot([xd(tk,6)], [yd(tk,6)], 'o');
 
        plot([xd(tk,7), xd(tk,8)], [yd(tk,7), yd(tk,8)],Color,'LineWidth',2*Bold+0.5);
        plot([xd(tk,7)], [yd(tk,7)], 'o');
 
        plot([xd(tk,8), xd(tk,9)], [yd(tk,8), yd(tk,9)],Color,'LineWidth',2*Bold+0.5);
        plot([xd(tk,8)], [yd(tk,8)], 'o');
 
        plot([xd(tk,9)], [yd(tk,9)], 'b*','LineWidth',2*Bold+0.5);
        title('Stick Diagram - 2D Plane Robot(9Dof)');        
end