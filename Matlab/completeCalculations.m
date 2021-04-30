close all
clear
clc

pi = sym(pi); %Better pi

[dh, baseHeight] = initDH(pi); %Kan ikke definere sym pi inne i funksjoner?

xVias = [0 0 0 0 0 0 0 0 0];
yVias = [60 -60 -198 -344 -21 245 335 202 60];
zVias = [-520 -520 -520 -520 -451 -477 -511 -520 -520];
phiVias = [48.9*pi/180 65.8*pi/180 86.9*pi/180 94.1*pi/180 55.1*pi/180 28.1*pi/180 28.8*pi/180 31.0*pi/180 48.9*pi/180]-pi/8;

thetaVias = invKinViasCalc(xVias,yVias,zVias,phiVias,dh,pi);
timeLim = [2 1 2 2 2 2 1.5 2];

velLim = 0.5;
velVias = velViasCalc(thetaVias, velLim);
accVias = zeros(length(timeLim)+1);
%% Inverse kinematics test plot
figure('Color','white','Name','Waypoint Control Plot')

dhMom = dh; % Making a momentary copy of dh
for i = 1:length(thetaVias(1,:))
    dhMom(2:5,2) = thetaVias(:,i);
    plotRobot(dhMom,baseHeight)
end
xlabel('x/[mm]')
ylabel('y/[mm]')
zlabel('z/[mm]')
[x, y] = meshgrid(-1000:1:1000);
z = zeros(size(x,1));
%surf(x,y,z, grey)

%% Path Calculation
[thetaFuncs, velFuncs, accFuncs] = pathCalcTot(thetaVias,timeLim,velVias,accVias);
timeStep = 1/10; %time steps for plot
[thetaDiscrete, timeLine] = pathDiscrete(thetaFuncs,timeLim,timeStep);

%% Body velocity
[XiMat] = BodyVel(dh, velFuncs);

%% Animated plotting
figure()

%Preallocate 
numFrames = length(timeLine);
frames = struct('cdata',cell(1,numFrames),'colormap',cell(1,numFrames));
dhMom = dh;
for i = 1:numFrames
    clf;        %Clearing plot values
    dhMom(2:5,2) =  thetaDiscrete(:,i);
    plotRobot(dhMom, baseHeight); 
    frames(i) = getframe(gcf);
end

video = VideoWriter('gaitAnimation1', 'MPEG-4');
video.FrameRate = 10;

open(video)
writeVideo(video,frames);  
close(video)

%% Plotting values for theta, velocity and acceleration
plotTheta(thetaFuncs,timeLim)
plotVel(velFuncs,timeLim)
plotAcc(accFuncs,timeLim)

%% Plotting yz-plane for end effector
figure('Color','white','Name','End Effector Coordinates')

numT = length(timeLine);
dhMom = dh;
endX = zeros(numT);
dhMom(2:5,2) =  thetaDiscrete(:,1);
T = symCalcT(dhMom);
y = T(2,4,7);
z = T(3,4,7);
plot(y, z,'ko')

hold on
grid
xlabel('y/[mm]')
ylabel('z/[mm]')

for i = 2:numT
    lastY = y;
    lastZ = z;
    dhMom(2:5,2) =  thetaDiscrete(:,i);
    T = symCalcT(dhMom);
    y = T(2,4,7);
    z = T(3,4,7);
    plot([lastY y], [lastZ z], 'k')
end