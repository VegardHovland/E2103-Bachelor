function [A] = realCalcA(a, alpha, d, theta, n)
%Creates the transfer matrix between two joints using Denavit-Hartenberg parameters

    A = zeros([4 4 n]);

    for i = 1:1:n
        A(:, :, i) = calcA(a(i), alpha(i), d(i), theta(i));
    end
end