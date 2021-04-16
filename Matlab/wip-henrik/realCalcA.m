function [A] = realCalcA(dh)
%Creates the transfer matrix between two joints using Denavit-Hartenberg parameters

    n = length(dh(:,1)); %Number of links
    A = zeros([4 4 n]);

    for i = 1:1:n
        d = dh(i,1);
        theta = dh(i,2);
        a = dh(i,3);
        alpha = dh(i,4);
        A(:, :, i) = calcA(d, theta, a, alpha);
    end
end