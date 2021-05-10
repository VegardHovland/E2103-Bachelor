close all
clear
clc

addpath('./quadruped_description/urdf/');

robot = importrobot('quadruped.urdf');
config = homeConfiguration(robot);      % Store starting pose
figure()
show(robot, 'visuals', 'on');
figure()
show(robot, config, 'visuals', 'on');



%configuration = randomConfiguration(robot);