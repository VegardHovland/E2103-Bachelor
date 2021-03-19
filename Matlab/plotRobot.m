function [] = plotRobot(T, n, baseHeight)
%PLOTROBOT Plotting robot in current configuration
    hold on
    grid on
    xlim([-800 700]);
    ylim([-800 700]);
    zlim([-400 600]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    
    for i=1:n
        x(i) = T(1,4,i);
        y(i) = T(2,4,i);
        z(i) = T(3,4,i);
        
        c = {'k' 'r' 'g' 'b' 'b' 'm' 'r'}; %Color for each joint and end effector
        plot3(x(i), y(i), z(i)+baseHeight, 'color', c{i}, 'marker', 'o') %Justerer for aksene til DH og høyden til stativet
        if i==2
            plot3([0 0],[0 y(i)],[baseHeight baseHeight], 'k');
            plot3([0 x(i)],[y(i) y(i)],[baseHeight z(i)+baseHeight], 'k');
        elseif i>1
            plot3([x(i-1) x(i)], [y(i-1) y(i)], [z(i-1)+baseHeight z(i)+baseHeight], 'k')
        end
        
    end
view([100 10])
end

