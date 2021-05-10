function [] = plotRobot(dh, baseHeight)
%PLOTROBOT Plotting robot in current configuration
    hold on
    grid on
    xlim([-400 400]);
    ylim([-400 400]);
    zlim([-50 550]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    
    T = symCalcT(dh);
    n = length(dh(:,1)); %number of links
    
    for i=1:n
        x(i) = T(1,4,i);
        y(i) = T(2,4,i);
        z(i) = T(3,4,i);
        
        c = {'k' 'r' 'm' 'b' 'b' 'c' 'g' 'k'}; %Color for each joint and end effector
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

