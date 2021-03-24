close all
clear
clc

pi=sym(pi);  

theta1 = pi/4;
theta2 = -pi/2;
theta3 = pi/6;

l1 = 100;
l2 = 100;
l3 = 100;

d           = [0 0 0];
theta       = [theta1 theta2 theta3];
a           = [l1 l2 l3];
alpha       = [0 0 0];
linkType    = ["rot" "rot" "rot" "rot"];

n = length(d);

A = symCalcA(a, alpha, d, theta, n);
T = symCalcT(A,n);

% Plot robot in starting positions
figure();
printRobot(T,n)

%%

x = a(1)*cos(theta(1))+a(2)*cos(theta(1)+theta(2))+a(3)*cos(theta(1)+theta(2)+theta(3));
y = a(1)*sin(theta(1))+a(2)*sin(theta(1)+theta(2))+a(3)*sin(theta(1)+theta(2)+theta(3));
phi = theta(1)+theta(2)+theta(3);
plot(x,y,'kx')

xm = x-a(3)*cos(phi);
ym = y-a(3)*sin(phi);

gamma =atan2(-ym/sqrt(xm^2+ym^2),-xm/sqrt(xm^2+ym^2));

theta(1) = gamma - acos(-(xm^2+ym^2+a(1)^2-(a(2)^2))/(2*a(1)*sqrt(xm^2+ym^2))); %+-acos
theta(2) = atan2((ym-a(1)*sin(theta(1)))/a(2),(xm-a(1)*cos(theta(1)))/a(2))-theta(1);
theta(3) = phi-(theta(1)+theta(2));

A = symCalcA(a, alpha, d, theta, n);
T = symCalcT(A,n);

figure();
printRobot(T,n)
