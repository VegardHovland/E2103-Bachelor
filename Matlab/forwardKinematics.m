close all
clear
clc

pi=sym(pi); % Acurate pi
syms theta1 theta2 theta3 theta4; %    for symbolic calulations

physicalProportions;

n=5; % 4dof + extra end-efector
m=6; % Antall posisjoner tegnet
height = baseHeight;

d = [60.8 0 0 0 0];                         % z-offset
%theta = [theta1 theta2 theta3 theta4 pi/6]; % Actuator angles
theta = [0 0 0 0 pi/6]; % Actuator angles
a = [73 femurLength tibiaLength tarsalLength phalangesLength-24];    % x-offset 
alpha = [-pi/2 0 0 0 0];                    % Dreining om x-akse
type = ["rot" "rot" "rot" "rot" "rot"];
%Endefector defined 24mm above surface

A = symCalcA(a, alpha, d, theta, n);
%A = simplify(A);
T = symCalcT(A,n);
%T = simplify(T);
J = symCalcJ(T, n, type);
%J = simplify(J);

%% Plot
figure()
printRobot(T,n, height);


%% Draw movement
figure()
theta = [linspace(-pi/12,pi/12,m)' linspace(pi/2,pi/4,m)' linspace(-2*pi/3,-2*pi/3,m)' linspace(pi/3,0,m)' linspace(pi/6,pi/6,m)']; % Actuator angles

for i = 1:m
    A = symCalcA(a, alpha, d, theta(i,:), n);
    T = symCalcT(A,n);
    printRobot(T,n, height);
    
end



