function [dh] = initDH()
%initDH creates a complete DH-table for this robot configuration
%   In this function, all parameters of the robot are defined and combined
%   into the Denevit-Hartenberg table. 
    
    pi = sym('pi');
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
    phalangesLength = legLength*phalangesProportions;
    phalangesAngle = -pi/3;                             % theta6 
    phalangesRadius = 24;
    baseHeight = legLength*baseHeightProportions;       % height of base link from ground
    
    % z-offset 
    d       = [baseHeight hipLength 0 0 0 0 phalangesLength-phalangesRadius];    
    % z-rotation, actuator angles
    theta   = [0 hipRotation 0 0 0 phalangesAngle 0]; 
    % x-rotation
    a       = [0 hipHeigth femurLength tibiaLength tarsalLength 0 0];
    % x-offset
    alpha   = [baseRotation hipRotation 0 0 0 tarsalRotation 0];
    % Type of rotation for the Jacobi-matrix
    linkType = ["rot" "rot" "rot" "rot" "rot" "rot" "rot"];

    dh = [d' theta' a' alpha' linkType'];
end