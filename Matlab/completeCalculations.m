%% Initialization of parameters and waypoints for the robot

close all
clear
clc
addpath('./initialization/');
addpath('./plotting/');
addpath('./kinematics/');
addpath('./pathPlanning/');

pi = sym(pi); %Better pi

[dh, baseHeight] = initDH(pi);

xVias = [0 0 0 0 0 0 0 0 0];
yVias = [60 -60 -198 -344 -21 245 335 202 60];
zVias = [-520 -520 -520 -520 -451 -477 -511 -520 -520];
phiVias = [48.9*pi/180 65.8*pi/180 86.9*pi/180 94.1*pi/180 55.1*pi/180 28.1*pi/180 28.8*pi/180 31.0*pi/180 48.9*pi/180]-pi/8;

thetaVias = invKinViasCalc(xVias,yVias,zVias,phiVias,dh,pi);
timeLim = [2 1 2 2 2 2 1.5 2];
moveType = ["lin" "lin" "lin" "quint" "quint" "quint" "quint" "lin"];

velLim = 0;
velVias = velViasCalc(thetaVias, velLim);
accVias = zeros(length(timeLim)+1);
%% Inverse kinematics test plot
% Plots the robot leg in its starting position to controll that the
% DH-table is correct
figure('Color','white','Name','Waypoint Control Plot')

dhMom = dh; % Making a momentary copy of dh
for i = 1:length(thetaVias(1,:))
    dhMom(2:5,2) = thetaVias(:,i);
    plotRobot(dhMom,baseHeight)
end
xlabel('x/[mm]')
ylabel('y/[mm]')
zlabel('z/[mm]')

%% Path Calculation
% Creates symbolic functions for the trajectories between all waypoints
[thetaFuncs, velFuncs, accFuncs] = pathCalcTot(thetaVias,timeLim,velVias,accVias);
timeStep = 1/10; %time steps for plot
[thetaDiscrete, timeLine] = pathDiscrete(thetaFuncs,timeLim,timeStep);

%% Body velocity 
% Attempt to create linear Cartesian movement for end effector
% Something is not working correctly, somehow double z values are applied
% to the Cartesian path. Only run this part for testing purposes.

%XiMat = bodyVel(dh, velFuncs); 
[thetaFuncs, velFuncs, accFuncs] = pathCalcTot(thetaVias,timeLim,velVias,accVias); % Gets quintic trajectories
timeStep = 1/10; %time steps for plot

qdot = tradjCalcLin([0 0],[60 -60],[-520 -520],2,dh);   % Calculates new velocity expressions for linear segment
q = int(qdot);                                          % Integral to find angle expresions
thetaFuncs(2:4,1) = q(3:5,1);                           % Replace the expression for the linear

[thetaDiscrete, timeLine] = pathDiscrete(thetaFuncs,timeLim,timeStep);

%% Animated plotting
% Creates a video of the movement between all waypoints and exports a video
plotAnimation(dh, baseHeight, thetaDiscrete, timeLine, 'gaitAnimation');

%% Plotting values for theta, velocity and acceleration
% Plots 3 figures with the values for all actuators
plotTheta(thetaFuncs,timeLim)
plotVel(velFuncs,timeLim)
plotAcc(accFuncs,timeLim)

%% Plotting yz-plane for end effector
plotEndEffector(dh,thetaDiscrete,timeLine);
