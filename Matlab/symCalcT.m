function [T] = symCalcT(A,n)
%symCalcT calculates all T matrices from base link to end efector for any
%robot configuration. By declaring A symbolic, the function enables
%symbolic calculation but it can be used numerically

    T = sym('T', [4 4 n]);  
    
    T(:,:,1) = A(:,:,1);
    for i = 2:1:n
        T(:,:,i) = T(:,:,i-1)*A(:,:,i);
    end
end

