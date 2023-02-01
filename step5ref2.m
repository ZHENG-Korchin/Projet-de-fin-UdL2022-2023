clear all 
clc;
m = 10;
gamma0 = 0.1;
gamma1= 0.35;
gamma2= 0.4;
gamma3= 0.15;
l = 0.3;
EI = 1.38 * 10^9 * 10^-3 * (6*10^-3)^3/12 ;
k1 = 3.51 * EI/l;
k2 = 2.99 * EI/l;
k3 = 2.58 * EI/l;
m1 = m * gamma1;
m2 = m * gamma2;
m3 = m * gamma3;
J1 = m1 * gamma1^2 * l^2/12;
J2 = m2 * gamma2^2 * l^2/12;
J3 = m3 * gamma3^2 * l^2/12;

t_step = 0.001;
t_f = 1;
t = 0: t_step : t_f;
N = size(t);
N = N(1,2);

x0 = zeros(6,1);
x = [x0, zeros(6,N-1)];
u = [0.001*sin(t);zeros(1,N); zeros(1,N); ];

theta_r = [pi/6; pi/6; pi/6];

for i = 1 : N
    xk = x(:,i); theta1 = xk(1); theta2= xk(2); theta3=xk(3); theta1_d = xk(4);theta2_d = xk(5);theta3_d = xk(6);
    uk = u(:,i);
    

    D11 = (gamma1^3/3 + gamma2^3/3 + gamma3^3/3 + gamma1^2 * gamma2 +gamma1^2 * gamma3 + gamma2^2 * gamma3 +...
        (gamma1 * gamma2^2 + 2* gamma1 * gamma2 * gamma3) * cos(theta2) + gamma2 * gamma3^2 * cos(theta3) + gamma1 * gamma3^2 * cos(theta2 + theta3)) * m*l^2;
        
    D22 = (gamma3^3/3 + gamma2^2 * gamma3 + gamma2 * gamma3^2 * cos(theta3)) * m*l^2;
    
    D33 = gamma3^3 * m * l^2/3;
    
    D12 = (gamma2^3/3 + gamma3^3/3 + gamma2^2 * gamma3 + (gamma1 * gamma2^2/2 + gamma1 * gamma2 * gamma3) * cos(theta2) + ...
        gamma2 * gamma3^2 * cos(theta3) + gamma1 * gamma3^2 * cos(theta2 + theta3)/2)* m * l^2;
    
    D21 = D12;
    
    D13 = (gamma3^3/3 + gamma2 * gamma3^2 *cos(theta3)/2 + gamma1 * gamma3^2 * cos(theta2 + theta3)/2 )*m*l^2;
    
    D31 = D13;
    
    D23 = (gamma3^3/3 + gamma2 * gamma3^2 * cos(theta3)/2) * m * l^2;
    
    D32 = D23;
    
    A = [D11, D12, D13; D21, D22, D23; D31, D32, D33];
    
    
    D111 = 0; D222 = 0; D333 = 0;
    
    D122 = - ( (gamma1*gamma2^2/2 + gamma1 * gamma2 * gamma3) * sin(theta2) + gamma1 * gamma3^3 * sin(theta2 + theta3)/2)*m*l^2;
    
    D133 = - (gamma2*gamma3^2*sin(theta3)/2 + gamma1*gamma3^2 * sin(theta2 + theta3)/2)*m*l^2;
    
    D211 = ( (gamma1 * gamma2^2/2 + gamma1 * gamma2 * gamma3)* sin(theta2) + gamma1 * gamma3^2 * sin(theta2 + theta3)/2)*m*l^2;
    
    D233 = -gamma2 * gamma3^2 * sin(theta3)*m*l^2/2;
    
    D311 = (gamma2*gamma3^2*sin(theta3)/2 + gamma1 * gamma3^2 *sin(theta2 + theta3)/2)*m*l^2;
    
    D322 = gamma2 * gamma3^2 * sin(theta3) *m *l^2/2;
    
    B = [D111, D122, D133; D211, D222, D233; D311, D322, D333];
    
    D112 = -((gamma1 * gamma2^2 + 2 * gamma1 * gamma2 * gamma3)* sin(theta2) + gamma1 * gamma3^2 * sin(theta2+ theta3)) * m * l^2;
    
    D113 = - (gamma2 * gamma3^2 * sin(theta3) + gamma1 * gamma3^2 * sin(theta2 + theta3))*m*l^2;
    
    D123 = D113;
    
    D213 = - gamma2 * gamma3^2 * sin(theta3) * m * l^2;
    
    D223 = D213;
    
    D312 = gamma2 * gamma3^2 * sin(theta3) * m * l^2;
    
    D212 = 0; D313 = 0; D323 = 0;
    
    C = [D112, D113, D123; D212, D213, D223; D312, D313, D323];
    
    D1 = gamma1 * EI/l * k1;
    D5 = gamma2 * EI/l * k2;
    D9 = gamma3 * EI/l * k3;
    D2 = 0; D3 = 0; D4 = 0; D6=0; D7 = 0; D8 = 0;
    
    D = [D1, D2, D3; D4, D5, D6; D7, D8, D9];
    
    
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