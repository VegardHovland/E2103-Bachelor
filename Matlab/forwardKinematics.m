close all
clear
clc

% Acurate pi
pi=sym(pi);
syms theta1 theta2 theta3 theta4;

n=5; % 4dof + extra end-efector
m=5; % Antall posisjoner tegnet
height = 575;

d = [60.8 0 0 0 0];                         % z-offset
%theta = [theta1 theta2 theta3 theta4 pi/6]; % Actuator angles
theta = [0 80*pi/180 -125*pi/180 80*pi/180 pi/6]; % Actuator angles
a = [73 256.178 284.418 105.361 55.361];    % x-offset 
alpha = [-pi/2 0 0 0 0];                    % Dreining om x-akse
%Endefector defined 24mm above surface

A = symCalcA(a, alpha, d, theta, n);
A = simplify(A);
T = symCalcT(A,n);
T = simplify(T);
J = symCalcJ(T, n, "rot");
J = simplify(J);

%% Plot
figure()
plot3(0, 0, 0+height, 'ko') %Ground point
hold on
grid
xlim([-800 700]);
ylim([-800 700]);
zlim([-200 600]);
printRobot(T,n, height);
view([100 10])


%% Draw movement
figure()
plot3(0, 0, 0+height, 'ko')
hold on
grid
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([-800 700]);
ylim([-800 700]);
zlim([-200 600]);

theta = [linspace(-pi/12,pi/12,m)' linspace(pi/2,pi/4,m)' linspace(-2*pi/3,-2*pi/3,m)' linspace(pi/3,0,m)' linspace(pi/6,pi/6,m)']; % Actuator angles

for i = 1:m
    A = symCalcA(a, alpha, d, theta(i,:), n);
    T = symCalcT(A,n);
    printRobot(T,n, height);
    
end
view([100 10])

