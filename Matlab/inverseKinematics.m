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
phi= pi/6;

theta(2) = atan2(x,-z)+hipRotation;

r = sqrt(x^2+z^2);
ym = y+d(2)+a(5)*sin(phi+theta(6))-d(7)*sin(pi/2-phi);
rm = r-a(2)-a(5)*cos(phi+theta(6))-d(7)*cos(pi/2-phi);

gamma = atan2(-ym/sqrt(rm^2+ym^2),-rm/sqrt(rm^2+ym^2));
theta(3) = gamma - acos(-(rm^2+ym^2+a(3)^2-a(4)^2)/(2*a(3)*sqrt(rm^2+ym^2)));

theta(4) = atan2((ym-a(3)*sin(theta(3)))/a(4),(rm-a(3)*cos(theta(3)))/a(4))-theta(3);
theta(5) = -(phi+theta(3)+theta(4)+theta(6));

figure();
dh = [d' theta' a' alpha' linkType'];

plotRobot(dh, baseHeight);

%% Generating angles for via points

num = 5;
x = [linspace(0,0,num) linspace(0,0,num)];
y = [linspace(150,-100,num) linspace(-100,-200,num)];
z = [linspace(-520,-520,num) linspace(-520,-420,num)];
phi = [linspace(pi/6,pi/4,num) linspace(pi/4,pi/3,num)];

m = length(x);

for i=1:m
    theta1(i) = 0;
    theta2(i) = atan2(x(i),-z(i))+hipRotation;
    
    r = sqrt(x(i)^2+z(i)^2);
    ym = y(i)+d(2)+a(5)*sin(phi(i)+theta(6))-d(7)*sin(pi/2-phi(i));
    rm = r-a(2)-a(5)*cos(phi(i)+theta(6))-d(7)*cos(pi/2-phi(i));
    gamma = atan2(-ym/sqrt(rm^2+ym^2),-rm/sqrt(rm^2+ym^2));
    
    theta3(i) = gamma - acos(-(rm^2+ym^2+a(3)^2-a(4)^2)/(2*a(3)*sqrt(rm^2+ym^2)));
    theta4(i) = atan2((ym-a(3)*sin(theta3(i)))/a(4),(rm-a(3)*cos(theta3(i)))/a(4))-theta3(i);
    theta5(i) = -(phi(i)+theta3(i)+theta4(i)+theta(6));
    theta6(i) = -pi/3;
    theta7(i) = 0;  
end
    
theta = [theta1' theta2' theta3' theta4' theta5' theta6' theta7'];

figure();
n = length(d);   
for i =1:m
    dh = [d' theta(i,:)' a' alpha' linkType'];
    plotRobot(dh, baseHeight);
end