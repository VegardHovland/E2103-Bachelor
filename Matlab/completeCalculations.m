close all
clear
clc

pi = sym(pi); %Better pi

[dh, baseHeight] = initDH(pi); %Kan ikke definere sym pi inne i funksjoner?

xVias = [0 0 0 0 0 0];
yVias = [150 -200 -500 0 200 150];
zVias = [-520 -520 -420 -360 -400 -520];
phiVias = [pi/6 pi/4 pi/2 pi/5 0 pi/6];

thetaVias = invKinViasCalc(xVias,yVias,zVias,phiVias,dh,pi);
timeLim = [3 1 2 2 2];

%% Inverse kinematics test plot
figure()
dhMom = dh; % Making a momentary copy of dh
for i = 1:length(thetaVias(1,:))
    dhMom(2:5,2) = thetaVias(:,i);
    plotRobot(dhMom,baseHeight)
end

%% Path Calculation
[thetaFuncs, velFuncs, accFuncs] = pathCalcTot(thetaVias,timeLim);

timeStep = 1/10; %time steps for plot
[thetaDiscrete, timeLine] = pathDiscrete(thetaFuncs,timeLim,timeStep);

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

video = VideoWriter('gaitAnimation', 'MPEG-4');
video.FrameRate = 10;

open(video)
writeVideo(video,frames);  
close(video)

%% Plotting values for theta, velocity and acceleration
close all
figure()
for i = 1:4
    subplot(2,2,i)
    fplot(thetaFuncs(i,1), [0 timeLim(1)])
    hold on
    fplot(thetaFuncs(i,2), [timeLim(1) timeLim(1)+timeLim(2)])
    fplot(thetaFuncs(i,3), [timeLim(1)+timeLim(2) timeLim(1)+timeLim(2)+timeLim(3)])
    fplot(thetaFuncs(i,4), [timeLim(1)+timeLim(2)+timeLim(3) timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4)])
    fplot(thetaFuncs(i,5), [timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4) timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4)+timeLim(5)])
    
    yticks([-double(pi)/2 -double(pi)/4 -double(pi)/8 0 double(pi)/8 double(pi)/4 double(pi)/2])
    yticklabels({'-pi/2', '-pi/4', '-pi/8', '0', 'pi/8', 'pi/4', 'pi'})
end
figure()
for i = 1:4
    subplot(2,2,i)
    fplot(velFuncs(i,1), [0 timeLim(1)])
    hold on
    fplot(velFuncs(i,2), [timeLim(1) timeLim(1)+timeLim(2)])
    fplot(velFuncs(i,3), [timeLim(1)+timeLim(2) timeLim(1)+timeLim(2)+timeLim(3)])
    fplot(velFuncs(i,4), [timeLim(1)+timeLim(2)+timeLim(3) timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4)])
    fplot(velFuncs(i,5), [timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4) timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4)+timeLim(5)])
    
    %yticks([-double(pi)/2 -double(pi)/4 -double(pi)/8 0 double(pi)/8 double(pi)/4 double(pi)/2])
    %yticklabels({'-pi/2', '-pi/4', '-pi/8', '0', 'pi/8', 'pi/4', 'pi'})
end
figure()
for i = 1:4
    subplot(2,2,i)
    fplot(accFuncs(i,1), [0 timeLim(1)])
    hold on
    fplot(accFuncs(i,2), [timeLim(1) timeLim(1)+timeLim(2)])
    fplot(accFuncs(i,3), [timeLim(1)+timeLim(2) timeLim(1)+timeLim(2)+timeLim(3)])
    fplot(accFuncs(i,4), [timeLim(1)+timeLim(2)+timeLim(3) timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4)])
    fplot(accFuncs(i,5), [timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4) timeLim(1)+timeLim(2)+timeLim(3)+timeLim(4)+timeLim(5)])
    
    %yticks([-double(pi)/2 -double(pi)/4 -double(pi)/8 0 double(pi)/8 double(pi)/4 double(pi)/2])
    %yticklabels({'-pi/2', '-pi/4', '-pi/8', '0', 'pi/8', 'pi/4', 'pi'})
end