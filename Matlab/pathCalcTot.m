function [thetaFuncs,velFuncs,accFuncs] = pathCalcTot(thetaVias,timeLimits,velVias,accVias)
%pathCalcTot calculatesthe total path functions for alle via points 
%   Using the tradjCalcQuint function, this function returns the symbolic
%   functions with respect to t for angle, velocity and acceleration for
%   all trajectories between via points.
   

    n = length(timeLimits);
    m = length(thetaVias(:,1));
    
    if ~exist('velVias','var')
     % Velocity parameters is not set, defaults to 0
      velVias = zeros(n+1);
    end
    if ~exist('accVias','var')
     % Acceleration parameters is not set, defaults to 0
      accVias = zeros(n+1);
    end

    timeLine = zeros(n+1);
    timeLine(1) = 0;
    
    thetaFuncs = sym('thetaFuncs', [m,n]);
    velFuncs = sym('velFuncs', [m,n]);
    accFuncs = sym('accFuncs', [m,n]);
    
    for i = 1:length(timeLimits)
        timeLine(i+1) = timeLine(i)+timeLimits(i);
    end
    
    for i=1:n           %Number of via points
        for j = 1:m     %Number of actuators
            [thetaFuncs(j,i), velFuncs(j,i), accFuncs(j,i)] = tradjCalcQuint([thetaVias(j,i) thetaVias(j,i+1)], [timeLine(i) timeLine(i+1)], [velVias(i) velVias(i+1)], [accVias(i) accVias(i+1)]);
        end
    end
end

