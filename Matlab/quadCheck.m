function [thetaOut] = quadCheck(thetaIn)
%QUANDRANTCHECK Summary of this function goes here
%   Detailed explanation goes here
    if thetaIn > pi
        thetaOut = thetaIn - 2*pi;
    elseif thetaIn < (-pi)
        thetaOut = thetaIn + 2*pi;
    else
        thetaOut = thetaIn;
    end
end

