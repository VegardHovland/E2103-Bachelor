function [thetaDiscrete, timeLine] = pathDiscrete(thetaFuncs,timeLim, timeStep)
%pathDiscrete creates an array of angles for each function in thetaFuncs
%   This function takes the symbolic functions in thetaFuncs and creates
%   angles given the time step. It also returns a timeline of all
%   timestamps where angles were calculated. 
    
    numVias = length(timeLim);
    numActuators = length(thetaFuncs(:,1));
    
    time = cell(1,numVias);
    time{1} = 0:timeStep:timeLim(1);
    numVals = length(time{1});
    for i = 2:numVias
        time{i} = time{i-1}(end):timeStep:time{i-1}(end)+timeLim(i);
        numVals = numVals + length(time{i});
    end
    
    thetaPath = cell(1,numVias);
    for i = 1:numVias
        numTimeSteps = length(time{i});
        thetaPath{i} = zeros(numActuators,numTimeSteps);
        for j = 1:numTimeSteps
            t = time{i}(j);
            thetaPath{i}(:,j) = subs(thetaFuncs(:,i), t);
        end
    end
    
    
    thetaDiscrete = thetaPath{1};
    timeLine = time{1};
    for i = 2:numVias
        thetaDiscrete = [thetaDiscrete thetaPath{i}];
        timeLine = [timeLine time{i}];
    end
end