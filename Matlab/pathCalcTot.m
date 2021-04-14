function [thetaFuncs,velFuncs,accFuncs] = pathCalcTot(thetaVias,timeLimits)
%pathCalcTot calculatesthe total path functions for alle via points 
%   Using the tradjCalcQuint function, this function returns the symbolic
%   functions with respect to t for angle, velocity and acceleration for
%   all trajectories between via points.
    
    n = length(timeLimits);
    m = length(thetaVias(:,1));

    timeLine = zeros(length(timeLimits)+1);
    timeLine(1) = 0;
    
    thetaFuncs = sym('thetaFuncs', [m,n]);
    velFuncs = sym('velFuncs', [m,n]);
    accFuncs = sym('accFuncs', [m,n]);
    
    for i = 1:length(timeLimits)
        timeLine(i+1) = timeLine(i)+timeLimits(i);
    end
    
    for i=1:n           %Number of via points
        for j = 1:m     %Number of actuators
            [thetaFuncs(j,i), velFuncs(j,i), accFuncs(j,i)] = tradjCalcQuint([thetaVias(j,i) thetaVias(j,i+1)], [timeLine(i) timeLine(i+1)]);
        end
    end
end

