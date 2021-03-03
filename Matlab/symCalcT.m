function [T] = symCalcT(A,n)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    T = sym('T', [4 4 n]);
    
    T(:,:,1) = A(:,:,1);
    for i = 2:1:n
        T(:,:,i) = T(:,:,i-1)*A(:,:,i);
    end

end

