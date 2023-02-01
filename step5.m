clear all 
clc;

load('step2.mat');
% m = 10;
% gama0 = 0.1;
% gama1= 0.35;
% gama2= 0.4;
% gama3= 0.15;
% L = 0.3;
% EI = 1.38 * 10^9 * 10^-3 * (6*10^-3)^3/12 ;
% k1 = 3.51 * EI/L;
% k2 = 2.99 * EI/L;
% k3 = 2.58 * EI/L;
% m1 = m * gama1;
% m2 = m * gama2;
% m3 = m * gama3;
% J1 = m1 * gama1^2 * L^2/12;
% J2 = m2 * gama2^2 * L^2/12;
% J3 = m3 * gama3^2 * L^2/12;

t_step = 0.001;
t_f = 1;
t = 0: t_step : t_f;
N = size(t);
N = N(1,2);

x0 = zeros(6,1);
x = [x0, zeros(6,N-1)];
u = [0.001*sin(t);zeros(1,N); zeros(1,N); ];    %力矩Tau

theta_r = [q1d(end); q2d(end); q3d(end)];   %

for i = 1 : N
    xk = x(:,i); 
    theta1 = xk(1); theta2= xk(2); theta3=xk(3); 
    theta1_d = xk(4);theta2_d = xk(5);theta3_d = xk(6);
    uk = u(:,i);
    

    Rparams4PRBDM =[];
    Rparams4PRBDM(1) = m ;
    Rparams4PRBDM(2) = L ;
    Rparams4PRBDM(3) = gama0 ;
    Rparams4PRBDM(4) = gama1;
    Rparams4PRBDM(5) = gama2;
    Rparams4PRBDM(6) = gama3;
    Rparams4PRBDM(7) = k1;
    Rparams4PRBDM(8) = k2;
    Rparams4PRBDM(9) = k3;
    Rparams4PRBDM(10) = I;
    [A,B,C,D] = PRBDM_3R_Dynamic_Matrices_Lagrange(xk(1:3),Rparams4PRBDM);
    
    %%
    
    f = [xk(4:6);
        pinv(A) * (-B * [theta1_d^2; theta2_d^2; theta3_d^2] - C * [theta1_d * theta2_d; theta1_d * theta3_d; theta2_d * theta3_d] - D* xk(1:3))];
        
    
    g = [zeros(3,3); pinv(A)];
    
    
    e = theta_r - xk(1:3); Kp = 10^2;
    e_d = - xk(4:6); Kd = 2* 10;
    
    PID = Kp * e + Kd * e_d;
    uk = B * [theta1_d^2; theta2_d^2; theta3_d^2] + C * [theta1_d * theta2_d; theta1_d * theta3_d; theta2_d * theta3_d] + D* xk(1:3) + A*PID;
    
    x(:,i+1) = xk  + t_step * (f + g * uk);
    
    
end

plot(x')
legend( 'q_1', 'q_2', 'q_3','q''_1', 'q''_2', 'q''_3');