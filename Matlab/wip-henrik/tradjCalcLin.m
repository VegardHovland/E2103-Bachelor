function [thetaFunc,velFunc,accFunc] = tradjCalcLin(theta, time, vel)
%tradjCalcLin Summary of this function goes here
%   Detailed explanation goes here

    t = sym('t');    
    thetaFunc = (theta(1)+theta(2)-vel*time(2))/2 + vel*t;
    velFunc = vel;
    accFunc = 0;
end

