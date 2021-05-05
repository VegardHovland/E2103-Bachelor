function [] = plotEndEffector(dh,thetaDiscrete,timeLine)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    figure('Color','white','Name','End Effector Coordinates')

    numT = length(timeLine);
    dhMom = dh;
    dhMom(2:5,2) =  thetaDiscrete(:,1);
    T = symCalcT(dhMom);
    y = T(2,4,7);
    z = T(3,4,7);
    plot(y, z,'ko')

    hold on
    grid
    xlabel('y/[mm]')
    ylabel('z/[mm]')

    for i = 2:numT
        lastY = y;
        lastZ = z;
        dhMom(2:5,2) =  thetaDiscrete(:,i);
        T = symCalcT(dhMom);
        y = T(2,4,7);
        z = T(3,4,7);
        plot([lastY y], [lastZ z], 'k')
    end
end

