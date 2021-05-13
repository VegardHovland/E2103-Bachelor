function [] = plotRobot(dh)
%PLOTROBOT Plotting robot in current configuration
    hold on
    grid on
    xlim([-400 400]);
    ylim([-400 400]);
    zlim([-500 550]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    T = symCalcT(dh);
    n = length(dh(:,1)); %number of links
    plot3(0,0,0,'ko');
    plot3([0,0], [0,0], [0,dh(1,1)], 'k--');
    for i=1:n
        x(i) = T(1,4,i);
        y(i) = T(2,4,i);
        z(i) = T(3,4,i);
        
        c = {'k' 'r' 'm' 'b' 'b' 'c' 'g' 'k'}; %Color for each joint and end effector
        plot3(x(i), y(i), z(i), 'color', c{i}, 'marker', 'o') %Justerer for aksene til DH og høyden til stativet
        
        %plot3([x(i-1) x(i)], [y(i-1) y(i)], [z(i-1) z(i)], 'k')
        
        
    end
view([100 10])
end

