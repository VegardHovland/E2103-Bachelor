yclear all
close all
clc

% Parameters and initial states
tf = 15;
g=9.81;

% Modify parameters and initial states
parameters =[; ; ; g];
state = [0;0;0;0];                

% Simulation
try

    %%%%%% MODIFY THE CODE AS YOU SEE FIT

    [tsim,xsim] = ode45(@(t,x)LegDynamics(t, x, parameters),[0,tf],state);

catch message
    display('Your simulation failed with the following message:')
    display(message.message)
    display(' ')

    % Assign dummy time and states if simulation failed
    tf = 0.1;
    tsim = [0,tf];
    xsim = 0;
end

