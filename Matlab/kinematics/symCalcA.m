function [A] = symCalcA(dh)
%symCalcA calculates all transfer matrices between two joints for the 
%entire robot configuration using Denavit-Hartenberg parameters. By 
%declaring "A" symbolic, the function enables symbolic calculation but it 
%can also be used numerically.

    n = length(dh(:,1)); %Number of links
    A = sym('A', [4 4 n]);
    
    for i = 1:1:n
        d = dh(i,1);
        theta = dh(i,2);
        a = dh(i,3);
        alpha = dh(i,4);
        A(:, :, i) = calcA(d, theta, a, alpha);
    end
end

