%% Initialization of DH-table using real properties
% Here the first row in the DH-table represent moving from the world frame
% to the base frame

close all
clear
clc
addpath('./init/');
addpath('./kinematics/');
addpath('./plotting/');

dh = initDH();
waypoints = initWaypoints(dh);

plotRobot(dh);