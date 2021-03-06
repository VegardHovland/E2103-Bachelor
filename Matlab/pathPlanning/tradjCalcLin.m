function [qdot] = tradjCalcLin(x,y,z,tLim, dh)
%tradjCalcLin calculates trajectories for linear Cartesian movement of the
%end effector

    t = sym('t');  
    dh(1,1) = 520;
    J = symCalcJ(dh);
    
    vx = (x(2)-x(1))*0;
    vy = (y(2)-y(1))/tLim;
    vz = (z(2)-z(1))*0;
    
    yFunc = y(1) + vy*t;
    %thetaxFunc = atan2(yFunc,z);
    
    wx = 1 / (1 + (yFunc / z(1))^2) * vy / y(1);
    wy = 0;
    wz = 0;
   
    xi = [vx vy vz wx wy wz]';
 
    qdot = pinv(J)*xi;  % Pseudo inverse to   
end

