function [] = printRobot(T,n,height)
%PRINTROBOT Printing configuration of 4dof robot

for i=1:n
    x(i) = T(1,4,i);
    y(i) = T(2,4,i);
    z(i) = T(3,4,i);
    plot3(y(i), -z(i), -x(i)+height, 'o') %Justerer for aksene til DH og høyden til stativet

    if i==1
        plot3([0 0],[0 -z(i)],[0+height 0+height], 'k');
        plot3([0 y(i)],[-z(i) -z(i)],[0+height -x(i)+height], 'k');
    else
        plot3([y(i-1) y(i)], [-z(i-1) -z(i)], [-x(i-1)+height -x(i)+height], 'k')
    end
end

