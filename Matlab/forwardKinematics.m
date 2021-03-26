close all
clear
clc

physicalProportions;                % Including robot physical dimensions

pi=sym(pi);                         % Acurate pi
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7;   % for symbolic calulations
syms a2 a3 a4 a5;
syms d2 d7;

%d2 = hipLength;
%d7 = phalangesLength-24;
%theta2 = 0;
%theta3 = 0;
%theta4 = 0;
%theta5 = 0;
%theta6 = -pi/3;
%a2 = hipHeigth;
%a3 = femurLength;
%a4 = tibiaLength;
%a5 = tarsalLength;
alpha1 = baseRotation;
alpha2 = hipRotation;
alpha6 = -pi/2;


d       = [0 d2 0 0 0 0 d7];                 % z-offset
theta   = [0 theta2+hipRotation theta3 theta4 theta5 theta6 0];      % z-rotation, actuator angles
a       = [0 a2 a3 a4 a5 0 0];   % x-offset
alpha   = [alpha1 alpha2 0 0 0 alpha6 0];                 % x-rotation
linkType = ["rot" "rot" "rot" "rot" "rot" "rot" "rot"];             % Type of rotation for the Jacobi-matrix

dh = [d' theta' a' alpha' linkType'];

n = length(d);                                                      % degrees of freedom + base rotation and end efector angle

A = symCalcA(a, alpha, d, theta, n);
A = simplify(A);
T = symCalcT(A,n);
T = simplify(T);
J = symCalcJ(T, n, linkType);
%J = simplify(J);

%% Plot robot in starting positions
figure();
plotRobot(T, n, baseHeight);

%% Mathmatical test
% Testing to make sure both forward kinematics calculations are correct

delta = pi/2+theta(6); %Theta(6) is allready negative so we use + here
r = a(2)+a(3)*cos(theta(3))+a(4)*cos(theta(3)+theta(4))+a(5)*cos(theta(3)+theta(4)+theta(5))+d(7)*cos(theta(3)+theta(4)+theta(5)+delta);
x = r*sin(theta2);
z = -r*cos(theta2)+baseHeight;
y = -d(2)+a(3)*sin(theta(3))+a(4)*sin(theta(3)+theta(4))+a(5)*sin(theta(3)+theta(4)+theta(5))+d(7)*sin(theta(3)+theta(4)+theta(5)+delta);

plot3(x,y,z,'kx');

%% Draw movement
m=6; % Antall posisjoner tegnet
figure()
theta = [linspace(0,0,m)' hipRotation+linspace(-pi/12,pi/12,m)' linspace(pi/2,pi/4,m)' linspace(-2*pi/3,-2*pi/3,m)' linspace(pi/3,0,m)' -linspace(pi/3,pi/3,m)' linspace(0,0,m)']; % Actuator angles

for i = 1:m
    A = symCalcA(a, alpha, d, theta(i,:), n);
    T = symCalcT(A,n);
    plotRobot(T,n, baseHeight);
    
end



