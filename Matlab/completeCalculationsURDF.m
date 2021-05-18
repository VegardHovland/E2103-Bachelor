%% Initialization of parameters and waypoints for the robot

close all
clear
clc
addpath('./initialization/');
addpath('./plotting/');
addpath('./kinematics/');
addpath('./pathPlanning/');
addpath('./urdf/quadruped_description/urdf/');

pi = sym(pi); %Better pi

[dh, baseHeight] = initDH(pi);

xVias = [0 0 0 0 0 0 0 0 0];
yVias = [60 -60 -198 -344 -21 245 335 202 60];
zVias = [-520 -520 -520 -520 -451 -477 -511 -520 -520];
phiVias = [48.9*pi/180 65.8*pi/180 86.9*pi/180 94.1*pi/180 55.1*pi/180 28.1*pi/180 28.8*pi/180 31.0*pi/180 48.9*pi/180]-pi/8;

thetaVias = invKinViasCalc(xVias,yVias,zVias,phiVias,dh,pi);
timeLim = [2 1 2 2 2 2 1.5 2];

velLim = 0.2;
velVias = velViasCalc(thetaVias, velLim);
accVias = zeros(length(timeLim)+1);

%% Importing URDF file
% All angles starting in 0 degrees

robot = importrobot('quadruped.urdf');
config = homeConfiguration(robot);      % Store starting pose

%% Path Calculation
% Creates symbolic functions for the trajectories between all waypoints

[thetaFuncs, velFuncs, accFuncs] = pathCalcTot(thetaVias,timeLim,velVias,accVias);
timeStep = 1/10; %time steps for plot
[thetaDiscrete, timeLine] = pathDiscrete(thetaFuncs,timeLim,timeStep);

%% Plot start position
% Plots complete robot in a starting position 

figure ('Color','white','Name','Robot Start Position','Position',[10 10 2010 1510])
startAngles = [0 -80 120 -70 0 80 -120 70 0 60 -100 -15 0 -60 100 15].*pi/180;

for i = 1:16
    config(i).JointPosition = double(startAngles(i));
end
show(robot, config);
xlim([-0.5 0.5]);
ylim([-0.75 0.75]);
zlim([-0.6 0.5]);
view([60 10])

%% Animation one leg

figure('Color','white','Name','Robot Gait','Position',[10 10 2010 1510])

numFrames = length(timeLine);
phase = 40;
frames = struct('cdata',cell(1,numFrames),'colormap',cell(1,numFrames));
for i = 1:numFrames
    clf;
    for j=2:4 
        config(j).JointPosition = -thetaDiscrete(j,i);
        if i<(numFrames-phase)
            config(j+4).JointPosition = thetaDiscrete(j,i+phase);
        else
            config(j+4).JointPosition = thetaDiscrete(j,i-numFrames+phase+1);
        end
    end
    
    show(robot, config, 'visuals', 'on');
    xlim([-0.5 0.5]);
    ylim([-0.75 0.75]);
    zlim([-0.6 0.5]);
    view([60 10])
    frames(i) = getframe(gcf);
end

video = VideoWriter('urdfTest', 'MPEG-4');
video.FrameRate = 10;

open(video)
writeVideo(video,frames);  
close(video)
