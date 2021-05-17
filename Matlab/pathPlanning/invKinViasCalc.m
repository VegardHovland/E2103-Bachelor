function [theta] = invKinViasCalc(xVias,yVias,zVias,phiVias,dh,pi)
%invKinViasCalc Creates viapoints using the inverse kinematics function
%   This function returns a matrix containing angles for each actuator in
%   each via point
    
    if ~exist('pi','var')
      pi = pi;
    end

    n = length(xVias);
    theta = sym('theta', [4 n]);
    for i = 1:n
        dhMom = invKinCalc(xVias(i),yVias(i),zVias(i),phiVias(i),dh,pi);
        theta(:,i) = dhMom(2:5,2);
    end   
end