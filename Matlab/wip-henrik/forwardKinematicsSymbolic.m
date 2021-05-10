syms d1 d2 d3 d4 d5 d6 d7;
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7;
syms a1 a2 a3 a4 a5 a6 a7;
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7;

d1 = 0;
d3 = 0;
d4 = 0;
d5 = 0;
d6 = 0;

theta1 = 0;
theta7 = 0;

a1 = 0;
a6 = 0;
a7 = 0;

alpha3 = 0;
alpha4 = 0;
alpha5 = 0;
alpha7 = 0;

d = [d1 d2 d3 d4 d5 d6 d7];
theta = [theta1 theta2 theta3 theta4 theta5 theta6 theta7];
a = [a1 a2 a3 a4 a5 a6 a7];
alpha = [alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7];

n = length(d); 

A = symCalcA(a, alpha, d, theta, n);
A = simplify(A);
T = symCalcT(A,n);
T = simplify(T);
J = symCalcJ(T, n, linkType);
%J = simplify(J);