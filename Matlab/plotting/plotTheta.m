function [] = plotTheta(thetaFuncs,timeLim)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    numFuncs = length(timeLim);
    figure('Color','white','Name','Theta Functions')
    %sgtitle('Actuator angles')
    for i = 1:4
        subplot(2,2,i)
        title(['\theta_' int2str(i+1)])
        xlabel('t/[s]')
        ylabel('\theta/[rad]')
        grid
        hold on
        box on
        if i == 1
            time = 0;
            for j = 1:numFuncs
                fplot(thetaFuncs(i,j)+pi/2, [time time+timeLim(j)])
                time = time + timeLim(j);
            end
        else
            time = 0;
            for j = 1:numFuncs
                fplot(thetaFuncs(i,j), [time time+timeLim(j)])
                time = time + timeLim(j);
            end
        end
        xlim([0 time]);
        %yticks([-double(pi)/2 -double(pi)/4 -double(pi)/6 -double(pi)/8 0 double(pi)/8 double(pi)/6 double(pi)/4 double(pi)/2])
        %yticklabels({'-pi/2', '-pi/4', '-pi/6', '-pi/8', '0', 'pi/8', 'pi/6', 'pi/4', 'pi/2'})
        
    end
    hold off
end

