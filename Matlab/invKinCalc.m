function [dh] = invKinCalc(x,y,z,phi,dh,pi)
%INVERSECALC Summary of this function goes here
%   Detailed explanation goes here
    
    theta2 = dh(2,2) + atan2(x,-z);
    if theta2>pi
        theta2 = theta2-2*pi;
    elseif theta2<-pi
        theta2 = theta2+2*pi;
    end
    dh(2,2) = theta2;
    
    r = sqrt(x^2+z^2);
    ym = y+dh(2,1)+dh(5,3)*sin(phi+dh(6,2))-dh(7,1)*sin(pi/2-phi);
    rm = r-dh(2,3)-dh(5,3)*cos(phi+dh(6,2))-dh(7,1)*cos(pi/2-phi);

    gamma = atan2(-ym/sqrt(rm^2+ym^2),-rm/sqrt(rm^2+ym^2));
    theta3 = gamma - acos(-(rm^2+ym^2+dh(3,3)^2-dh(4,3)^2)/(2*dh(3,3)*sqrt(rm^2+ym^2)));
    if theta3>pi
        theta3 = theta3-2*pi;
    elseif theta3<-pi
        theta3 = theta3+2*pi;
    end
    dh(3,2) = theta3;
    
    theta4 = atan2((ym-dh(3,3)*sin(dh(3,2)))/dh(4,3),(rm-dh(3,3)*cos(dh(3,2)))/dh(4,3))-dh(3,2);
    if theta4>pi
        theta4 = theta4-2*pi;
    elseif theta4<-pi
        theta4 = theta4+2*pi;
    end
    dh(4,2) = theta4;
    
    theta5 = -(phi+dh(3,2)+dh(4,2)+dh(6,2));
    if theta5>pi
        theta5 = theta5-2*pi;
    elseif theta5<-pi
        theta5 = theta5+2*pi;
    end
    dh(5,2) = theta5;
end

