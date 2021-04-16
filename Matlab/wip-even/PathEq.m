function [acc, v, alpha] = PathEq(alpha_0, alpha_f, t_0, t_f)

syms t;

%inital values
acc_0 = 0;
v_0 = 0;

%stopvalues
acc_f = 0;
v_f = 0;

%finding a-matrix, using formula
c=[alpha_0; v_0; acc_0; alpha_f; v_f; acc_f];
b=[1 t_0 t_0^2 t_0^3 t_0^4 t_0^5; 0 1 2*t_0 3*t_0^2 4*t_0^3 5*t_0^4; 0 0 2 6*t_0 12*t_0^2 20*t_0^3; 1 t_f t_f^2 t_f^3 t_f^4 t_f^5; 0 1 2*t_f 3*t_f^2 4*t_f^3 5*t_f^4; 0 0 2 6*t_f 12*t_f^2 20*t_f^3];

a=inv(b)*c;

%Path equations
alpha = a(1) + a(2)*t + a(3)*t^2 + a(4)*t^3 + a(5)*t^4 + a(6)*t^5;
v = a(2) + 2*a(3)*t + 3*a(4)*t^2 + 4*a(5)*t^3 + 5*a(6)*t^4;
acc = 2*a(3) + 6*a(4)*t + 12*a(5)*t^2 + 20*a(6)*t^3;

end



