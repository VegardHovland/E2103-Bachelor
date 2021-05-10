function [] = plotAnimation(dh, baseHeight, thetaDiscrete, timeLine, fileName)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

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

    video = VideoWriter(fileName, 'MPEG-4');
    video.FrameRate = 10;

    open(video)
    writeVideo(video,frames);  
    close(video)
end

