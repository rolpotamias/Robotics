function []=Plot_results(qd,apost1,apost2)
    figure;
    subplot(3,3,1);
    plot(smooth(qd(:,1))); xlim([0 1000]);
    title ('Angle Joint 1');

    subplot(3,3,2);
    plot(smooth(qd(:,2))); xlim([0 1000]);
    title ('Angle Joint 2');

    subplot(3,3,3);
    plot(smooth(qd(:,3))); xlim([0 1000]);
    title ('Angle Joint 3');

    subplot(3,3,4);
    plot(smooth(qd(:,4))); xlim([0 1000]);
    title ('Angle Joint 4');

    subplot(3,3,5);
    plot(smooth(qd(:,5))); xlim([0 1000]);
    title ('Angle Joint 5');

    subplot(3,3,6);
    plot(smooth(qd(:,6))); xlim([0 1000]);
    title ('Angle Joint 6');

    subplot(3,3,7);
    plot(smooth(qd(:,7))); xlim([0 1000]);
    title ('Angle Joint 7');

    subplot(3,3,8);
    plot(smooth(qd(:,8))); xlim([0 1000]);
    title ('Angle Joint 8');

    subplot(3,3,9);
    plot(smooth(qd(:,9))); xlim([0 1000]);
    title ('Angle Joint 9');
    figure;
    title('Joint Distance from Obastacles'); subplot(2,1,1);plot(apost1(:,3:8));  xlabel('Time'); ylabel('Distance from Obstacle No1');legend('Joint 4' , 'Joint 5','Joint 6','Joint 7','Joint 8','Joint 9'); subplot(2,1,2);  plot(apost2(:,3:8)); legend('Joint 4' , 'Joint 5','Joint 6','Joint 7','Joint 8','Joint 9'); xlabel('Time'); ylabel('Distance from Obstacle No2');
end