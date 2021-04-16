function [J] = realCalcJ(T, n, type)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Jv = zeros([3 n]);
Jw = zeros([3 n]);

for i = 1:n
    if lower(type(i)) == "rot"
        if i == 1
            Jv(1:3, i) = cross([0;0;1],(T(1:3, 4, n) - [0;0;0]));
            Jw(1:3, i) = [0;0;1];
        else
            Jv(1:3, i) = cross(T(1:3, 3, i-1),(T(1:3, 4, n) - (T(1:3, 4, i-1))));
            Jw(1:3, i) = T(1:3, 3, i-1);
        end     
    else
        if i == 1
            Jv(1:3, i) = [0;0;0];
        else
            Jv(1:3, i) = T(1:3, 3, i-1);
        end      
        Jw(i,1:3) = 0;
    end
end

J = [Jv;Jw];
end