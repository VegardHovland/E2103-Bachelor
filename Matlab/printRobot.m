function [] = printRobot(T,n,height)
%PRINTROBOT Printing configuration of 4dof robot

plot3(0, 0, 0+height, 'ko') %Ground point
hold on
grid on
xlim([-800 700]);
ylim([-800 700]);
zlim([-400 600]);

for i=1:n-1
    x(i) = T(1,4,i);
    y(i) = T(2,4,i);
    z(i) = T(3,4,i);
    c = {'r' 'g' 'b' 'm' 'k'}; %Color for each joint and end effector
    plot3(x(i), y(i), z(i)+height, 'color', c{i}, 'marker', 'o') %Justerer for aksene til DH og høyden til stativet

    if i==1
        plot3([0 0],[0 y(i)],[0+height 0+height], 'k');
        plot3([0 x(i)],[y(i) y(i)],[0+height z(i)+height], 'k');
    elseif 
        plot3([x(i-1) x(i)], [y(i-1) y(i)], [z(i-1)+height z(i)+height], 'k')
    end
end
view([100 10])

