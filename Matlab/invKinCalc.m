function [dh] = invKinCalc(x,y,z,phi,dh,pi)
%invKinCalc calculates robot angles given desired values for x, y, z, phi 
%   This function returns a full Denavit-Hartenberg table for all momentary
%   values at the current stance.

    if ~exist('pi','var')
      pi = pi;
    end

    theta2 = dh(2,2) + atan2(x,-z);
    dh(2,2) = quadCheck(theta2);
    
    r = sqrt(x^2+z^2);
    ym = y+dh(2,1)+dh(5,3)*sin(phi+dh(6,2))-dh(7,1)*sin(pi/2-phi);
    rm = r-dh(2,3)-dh(5,3)*cos(phi+dh(6,2))-dh(7,1)*cos(pi/2-phi);

    gamma = atan2(-ym/sqrt(rm^2+ym^2),-rm/sqrt(rm^2+ym^2));
    theta3 = gamma - acos(-(rm^2+ym^2+dh(3,3)^2-dh(4,3)^2)/(2*dh(3,3)*sqrt(rm^2+ym^2)));
    dh(3,2) = quadCheck(theta3);
    
    theta4 = atan2((ym-dh(3,3)*sin(dh(3,2)))/dh(4,3),(rm-dh(3,3)*cos(dh(3,2)))/dh(4,3))-dh(3,2);
    dh(4,2) = quadCheck(theta4);
    
    theta5 = -(phi+dh(3,2)+dh(4,2)+dh(6,2));
    dh(5,2) = quadCheck(theta5);
end

