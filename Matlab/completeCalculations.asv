close all
clear
clc

pi = sym(pi); %Better pi

[dh, baseHeight] = initDH(pi); %Kan ikke definere sym pi inne i funksjoner?

xVias = [0 0 0 0 0 0];
yVias = [150 -200 -400 0 200 150];
zVias = [-520 -520 -420 -360 -400 -520];
phiVias = [pi/6 pi/4 pi/2 pi/5 0 pi/6];

thetaVias = invKinViasCalc(xVias,yVias,zVias,phiVias,dh,pi);
timeLim = [3 1 2 2 2];


%% Inverse kinematics test plot
figure()
dhMom = dh;
for i = 1:length(thetaVias(1,:))
    dhMom(2:5,2) = thetaVias(:,i);
    plotRobot(dhMom,baseHeight)
end


%%
thetaFuncs = pathCalcTot(thetaVias,timeLim);


n = 1; %time divisions to plot
[thetaDiscrete, timeLine] = pathDiscrete(thetaFuncs,timeLim,n);

figure()
dhMom = dh;
for i = 1:length(timeLine)
    dhMom(2:5,2) =  thetaDiscrete(:,i);
    plotRobot(dhMom, baseHeight); 
end







