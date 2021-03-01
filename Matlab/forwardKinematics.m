close all
clear
clc

% Acurate pi
pi=sym(pi);
syms theta1 theta2 theta3 theta4;

theta = [theta1 theta2 theta3 theta4];  % Actuator angles
a = [0 1 1 1];                          % x-offset
alpha = [-pi/2 0 0 0];                  % Dreining om x-akse
d = [1 0 0 0];                          % z-offset