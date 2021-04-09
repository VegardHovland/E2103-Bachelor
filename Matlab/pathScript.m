close all
clear
clc
%define  angles
theta_2 = [30 20 45 50];
theta_3 = [59 30 43 -9];
theta_4 = [30 2 4 -20];
theta_5 = [40 32 21 0];

%time limitations in seconds
t_1 = 2;
t_2 = 3;
t_3 = 1;
t_sum = t_1 + t_2 + t_3

%collecting data from function:

%theta_2 angle trajectory
[acc_theta_2_1, v_theta_2_1, alpha_theta_2_1]=PathEq(theta_2(1), theta_2(2), 0, t_1);
[acc_theta_2_2, v_theta_2_2, alpha_theta_2_2]=PathEq(theta_2(2), theta_2(3), t_1, t_1 + t_2);
[acc_theta_2_3, v_theta_2_3, alpha_theta_2_3]=PathEq(theta_2(3), theta_2(4), t_1 + t_2, t_sum);

%#copy method for theta_2 trajectory
[acc_theta_3, v_theta_3, alpha_theta_3]=PathEq(theta_3(1), theta_3(2),0 , t_1);
[acc_theta_4, v_theta_4, alpha_theta_4]=PathEq(theta_4(1), theta_4(2), 0,  t_1);
[acc_theta_5, v_theta_5, alpha_theta_5]=PathEq(theta_5(1), theta_5(2), 0, t_1);

%plotting of theta_5 values
fplot(alpha_theta_2_1, [0, t_1])
hold on
fplot(alpha_theta_2_2, [t_1, t_1+t_2])
fplot(alpha_theta_2_3, [t_1 + t_2, t_sum])
xlabel('time(sec)')
ylabel('angle(degrees)')
title('Angle')

%%

figure
fplot(v_theta_2, [0, t_1])
xlabel('time(sec)')
ylabel('Velocity(m/s)')
title('velocity')

figure
fplot(acc_theta_2, [0, t_1])
xlabel('time(sec)')
ylabel('acceleration(m/s^2)')
title('acceleration')
