function [velVias] = velViasCalc(thetaVias,velLim)
%VELVIASCALC Summary of this function goes here
%   Detailed explanation goes here
    [numActuators, numVias] = size(thetaVias);
    velVias = zeros(numActuators, numVias);
    
    for i = 1:numActuators
        for j = 2:numVias-1
            if thetaVias(i,j-1) > thetaVias(i,j) && thetaVias(i,j) > thetaVias(i,j+1)
                velVias(i,j) = -velLim;
            elseif thetaVias(i,j-1) < thetaVias(i,j) && thetaVias(i,j) < thetaVias(i,j+1)
                velVias(i,j) = velLim;
            else
                velVias(i,j) = 0;
            end
        end
    end
        
end

