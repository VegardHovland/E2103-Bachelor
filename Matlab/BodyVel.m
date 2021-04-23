function [XiMat] = BodyVel(dh, velFuncs)

J = symCalcJ(dh);
[l, k] = size(velFuncs);

%XiMat = zeros(6, 5);%length(k));
%Xi = zeros(6, 1);

for i = 1:5%length(k)
    
    VelVec =[0; velFuncs(:, i); 0; 0];
    
    Xi = J*VelVec;

    for j = 1:6
       XiMat(j,i) = Xi(j);
    end
end
%XiMat
%Xi = [zeros(length(VelFuncs), 1)]