function [] = plotRobot(T, n)
%PLOTROBOT Plotting 2d robot in current configuration
    hold on
    grid on
    xlim([-350 350]);
    ylim([-350 350]);
    xlabel('X');
    ylabel('Y');

    
    plot(0, 0, 'ko')
    for i=1:n
        x(i) = T(1,4,i);
        y(i) = T(2,4,i);
             
        c = {'r' 'g' 'b'};
        plot(x(i), y(i), 'color', c{i}, 'marker', 'o')
        plot([0 x(1)], [0 y(1)], 'k')
        if i>1
            plot([x(i-1) x(i)], [y(i-1) y(i)], 'k')
        end      
    end
end

