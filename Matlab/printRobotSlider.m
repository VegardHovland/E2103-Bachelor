function [] = printRobotSlider(H,n,m,height)

k = 1;

S.fh = figure('units','pixels',...
              'position',[100 100 800 400],...
              'menubar','none',...
              'name','slider_plot',...
              'numbertitle','off',...
              'resize','off');  
S.ax = axes('unit','pix',...
            'position',[50 80 700 300]);
        
hold on
grid on
xlim([-800 700]);
ylim([-800 700]);
zlim([-400 600]);

plot3(0, 0, 0+height, 'ko') %Ground point

view([100 10])

c = {'r' 'g' 'b' 'm' 'k'}; %Color for each joint and end effector

%Justerer for aksene til DH og høyden til stativet
S.J2 = plot3(H(2,4,1,k), -H(3,4,1,k), -H(1,4,1,k)+height, 'color', c{1}, 'marker', 'o');
S.J3 = plot3(H(2,4,2,k), -H(3,4,2,k), -H(1,4,2,k)+height, 'color', c{2}, 'marker', 'o');
S.J4 = plot3(H(2,4,3,k), -H(3,4,3,k), -H(1,4,3,k)+height, 'color', c{3}, 'marker', 'o');
S.J5 = plot3(H(2,4,4,k), -H(3,4,4,k), -H(1,4,4,k)+height, 'color', c{4}, 'marker', 'o');
S.J6 = plot3(H(2,4,5,k), -H(3,4,5,k), -H(1,4,5,k)+height, 'color', c{5}, 'marker', 'o');


S.sl = uicontrol('style','slide',...
                 'unit','pix',...
                 'position',[50 10 700 30],...
                 'min',1,'max',m,'val',1,...
                 'sliderstep',[1/(m-1) 1/(m-1)],...
                 'callback',{@sl_call,S}); 
             
function [] = sl_call(varargin)
% Callback for the slider.
[h,S] = varargin{[1,3]};  % calling handle and data structure.
x2 = H(2,4,1,round(get(h,'value')));
y2
z2
set(S.J2,'xdata',round(get(h,'value')),'ydata',round(get(h,'value')), 'zdata', round(get(h,'value')))