close all
clear
clc

pi=sym(pi);                         % Acurate pi
%syms theta1 theta2 theta3 theta4 theta5 theta6 theta7;   % for symbolic calulations


physicalProportions;                % Including robot physical dimensions

theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;

d       = [0 hipLength 0 0 0 0 phalangesLength-24];                 % z-offset
theta   = [0 theta2+hipRotation theta3 theta4 theta5 -pi/3 0];      % z-rotation, actuator angles
a       = [0 hipHeigth femurLength tibiaLength tarsalLength 0 0];   % x-offset
alpha   = [baseRotation hipRotation 0 0 0 -pi/2 0];                 % x-rotation
linkType = ["rot" "rot" "rot" "rot" "rot" "rot" "rot"];             % Type of rotation for the Jacobi-matrix

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
zLength = a(2)+a(3)+a(4)+a(5)+d(7)*cos(pi/6);
x = zLength*sin(theta2);
z = -zLength*cos(theta2)+baseHeight;
y = -d(2)+a(3)*sin(theta(3))+a(4)*sin(theta(3)+theta(4))+a(5)*sin(theta(3)+theta(4)+theta(5))+d(7)*sin(theta(3)+theta(4)+theta(5)+pi/2-theta(6));

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



