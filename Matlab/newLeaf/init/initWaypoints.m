function [waypoints] = initWaypoints(dh)
%INITWAYPOINTS Summary of this function goes here
%   Detailed explanation goes here
    
    pi = sym('pi');

    xVias = [0 0 0 0 0 0 0 0 0];
    yVias = [60 -60 -198 -344 -21 245 335 202 60];
    zVias = [-520 -520 -520 -520 -451 -477 -511 -520 -520];
    phiVias = [48.9*pi/180 65.8*pi/180 86.9*pi/180 94.1*pi/180 55.1*pi/180 28.1*pi/180 28.8*pi/180 31.0*pi/180 48.9*pi/180]-pi/8;

    waypoints = [xVias; yVias; xVias; phiVias];
end

