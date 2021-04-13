function [theta] = invKinViasCalc(xVias,yVias,zVias,phiVias,dh,pi)
%INVKINVIASCALC Creates viapoints in radians using the inverse kinematics
%function
%   Detailed explanation goes here
    
    n = length(xVias);
    theta = sym('theta', [4 n]);
    for i = 1:n
        dhMom = invKinCalc(xVias(i),yVias(i),zVias(i),phiVias(i),dh,pi);
        theta(:,i) = dhMom(2:5,2);
    end   
end

