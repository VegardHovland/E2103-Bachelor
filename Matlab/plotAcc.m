function [] = plotAcc(accFuncs,timeLim)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    numFuncs = length(timeLim);
    figure('Color','white','Name','Acceleration Functions')
    for i = 1:4
        subplot(2,2,i)
        title(['\alpha_' int2str(i+1)])
        xlabel('t/[s]')
        ylabel('\alpha/[rad/s^2]')
        grid
        hold on
        box on
        if i == 1
            time = 0;
            for j = 1:numFuncs
                fplot(accFuncs(i,j)+pi/2, [time time+timeLim(j)])
                time = time + timeLim(j);
            end
        else
            time = 0;
            for j = 1:numFuncs
                fplot(accFuncs(i,j), [time time+timeLim(j)])
                time = time + timeLim(j);
            end
        end
        xlim([0 time]);
    end
    hold off
end