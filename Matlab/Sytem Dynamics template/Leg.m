clear all
clc

%Run this fisrt to generate symbolic matrices

% Parameters
syms J M R g To real
% Variables
syms  theta1 theta2 theta3 theta4 theta5 theta6 theta7 real
syms dtheta1 dtheta2 dtheta3 dtheta4 dtheta5 dtheta6 dtheta7 real

% Define symbolic variable q for the generalized coordinates
% x and theta
q  = [theta1; theta2; theta3; theta4; theta5; theta6; theta7];
% Define symbolic variable dq for the derivatives 
% of the generalized coordinates
dq = [dtheta1; dtheta2; dtheta3; dtheta4; dtheta5; dtheta6; dtheta7];
% Write the expressions for the position of
% the links take this from kinematics:
p = [];   

% Kinetic energy
T = ; % kinetic energy equations provided in thesis

dp = jacobian(p,q)*dq; % calc velocity
T  = T + 1/2*M*dp'*dp; % add linear kinetic energy

I     = ;                           % inertia in rotation matrices provided in seperate matlab file
omega = ;                           % angular velocity eq 4.27

T  = T + 1/2*I*omega'*omega;        % add rotational kinetic energy, eq 4.29

T = simplify(T);

% Potential energy
V =  ;                              % Eq 4.3 in thesis paper

% Generalized forces
Q = [0;To];                         % Edit this to fit eq 4.32

% Lagrangian
Lag = T - V ;                       % Eq. 4.3

Lag_q = simplify(jacobian(Lag,q)).';
Lag_qdq = simplify(jacobian(Lag_q.',dq));
Lag_dq = simplify(jacobian(Lag,dq)).';
Lag_dqdq = simplify(jacobian(Lag_dq.',dq));

% The equations have the form W*q_dotdot = RHS, with
W = Lag_dqdq;
RHS = Q + simplify(Lag_q - Lag_qdq*dq);

state = [q;dq];
param = [J; M; R; g];

matlabFunction(p, 'file','LegPosition','vars',{state,param});
matlabFunction(W,RHS, 'file','LegODEMatrices','vars',{state,To,param});
