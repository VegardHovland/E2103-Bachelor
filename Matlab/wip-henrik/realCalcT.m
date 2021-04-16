function [T] = realCalcT(dh)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    A = realCalcA(dh);

    n = length(A(1,1,:)); %number of links
    T = zeros([4 4 n]);
    
    T(:,:,1) = A(:,:,1);
    for i = 2:1:n
        T(:,:,i) = T(:,:,i-1)*A(:,:,i);
    end
end

