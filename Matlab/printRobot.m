function [] = printRobot(T,n,height)
%PRINTROBOT Printing configuration of 4dof robot
hold on
plot3(0, 0, 0+height, 'ko') %Ground point

grid on
xlim([-800 700]);
ylim([-800 700]);
zlim([-400 600]);

for i=1:n
    x(i) = T(1,4,i);
    y(i) = T(2,4,i);
    z(i) = T(3,4,i);
    c = {'r' 'g' 'b' 'm' 'k'}; %Color for each joint and end effector
    plot3(y(i), -z(i), -x(i)+height, 'color', c{i}, 'marker', 'o') %Justerer for aksene til DH og h�yden til stativet

    if i==1
        plot3([0 0],[0 -z(i)],[0+height 0+height], 'k');
        plot3([0 y(i)],[-z(i) -z(i)],[0+height -x(i)+height], 'k');
    else
        plot3([y(i-1) y(i)], [-z(i-1) -z(i)], [-x(i-1)+height -x(i)+height], 'k')
    end
endtheta2
view([100 10])

