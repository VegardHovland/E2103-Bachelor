function [dh, baseHeight] = initDH(pi)
%INITIALISEDH Summary of this function goes here
%   Detailed explanation goes here
    
    legLength = 800;

    femurProportions = 0.329;
    tibiaProportions = 0.3643;
    tarsalProportions = 0.2009;
    phalangesProportions = 0.1059;
    baseHeightProportions = 0.65;

    baseRotation = pi/2;                                % alpha1
    hipLength = 60.8;                                   % d2
    hipHeigth = 73;                                     % a2
    hipRotation = -pi/2;                                % alpha2
    tarsalRotation = -pi/2;                             % alpha6
    femurLength = legLength*femurProportions;           % a3
    tibiaLength = legLength*tibiaProportions;           % a4
    tarsalLength = legLength*tarsalProportions;         % a5
    phalangesLength = legLength*phalangesProportions;   % a6
    phalangesAngle = -pi/3;                             % 
    phalangesRadius = 24;
    baseHeight = legLength*baseHeightProportions;       % height of base link from ground
    
    d       = [0 hipLength 0 0 0 0 phalangesLength-phalangesRadius];                                    % z-offset 
    theta   = [0 hipRotation 0 0 0 phalangesAngle 0]; % z-rotation, actuator angles
    a       = [0 hipHeigth femurLength tibiaLength tarsalLength 0 0];                                  % x-offset
    alpha   = [baseRotation hipRotation 0 0 0 tarsalRotation 0];                       % x-rotation
    linkType = ["rot" "rot" "rot" "rot" "rot" "rot" "rot"];         % Type of rotation for the Jacobi-matrix

    dh = [d' theta' a' alpha' linkType'];
end