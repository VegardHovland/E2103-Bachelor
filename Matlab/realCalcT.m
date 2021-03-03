function [T] = realCalcT(A,n)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    T = zeros([n n n]);
    
    T(:,:,1) = A(:,:,1);
    for i = 2:1:n
        T(:,:,i) = T(:,:,i-1)*A(:,:,i);
    end

end

