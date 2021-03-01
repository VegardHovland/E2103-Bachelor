close all
clear
clc

% Acurate pi
pi=sym(pi);
syms theta1 theta2 theta3 theta4;

n=5; % 4dof + extra end-efector

d = [60.8 0 0 0 0];                         % z-offset
theta = [theta1 theta2 theta3 theta4 pi/6]; % Actuator angles
a = [73 256.178 284.418 105.361 55.361];    % x-offset 
alpha = [-pi/2 0 0 0 0];                    % Dreining om x-akse

%Endefector defined 24mm above surface
%A = symCalcA(a, alpha, d, theta, n)
%noe funker ikke
