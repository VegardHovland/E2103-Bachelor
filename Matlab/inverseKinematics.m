close all
clear
clc

pi=sym(pi);  

physicalProportions;

d       = [0 hipLength 0 0 0 0 phalangesLength-24];                 % z-offset
theta = [0 hipRotation 0 0 0 -pi/3 0];                                        % z-rotation, actuator angles
a       = [0 hipHeigth femurLength tibiaLength tarsalLength 0 0];   % x-offset
alpha   = [baseRotation hipRotation 0 0 0 -pi/2 0];                 % x-rotation
linkType = ["rot" "rot" "rot" "rot" "rot" "rot" "rot"];             % Type of rotation for the Jacobi-matrix

% Desired configuration
x = 0;
y = 100;
z = -520;
phi= pi/3;

theta(2) = atan2(x,-z)+hipRotation;
r = sqrt(x^2+z^2);

ym = y+d(2)+a(5)*sin(phi+theta(6))-d(7)*sin(pi/2-phi);
rm = r-a(2)-a(5)*cos(phi+theta(6))-d(7)*cos(pi/2-phi);

gamma = atan2(-ym/sqrt(rm^2+ym^2),-rm/sqrt(rm^2+ym^2));

theta(3) = gamma - acos(-(rm^2+ym^2+a(3)^2-a(4)^2)/(2*a(3)*sqrt(rm^2+ym^2)));
theta(4) = atan2((ym-a(3)*sin(theta(3)))/a(4),(rm-a(3)*cos(theta(3)))/a(4))-theta(3);
theta(5) = -(phi+theta(3)+theta(4)+theta(6));

n = length(d);                                                      % degrees of freedom + base rotation and end efector angle
A = symCalcA(a, alpha, d, theta, n);
T = symCalcT(A,n);


figure();
plotRobot(T, n, baseHeight);

