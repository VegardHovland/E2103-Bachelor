function [A] = symCalcA(a, alpha, d, theta, n)
%symCalcA calculates all transfer matrices between two joints for the 
%entire robot configuration using Denavit-Hartenberg parameters. By 
%declaring "A" symbolic, the function enables symbolic calculation but it 
%can also be used numerically.

    A = sym('A', [4 4 n]);

    for i = 1:1:n
        A(:, :, i) = calcA(a(i), alpha(i), d(i), theta(i));
    end
end

