function [A,B,C,D] = PRBDM_3R_Dynamic_Matrices_Lagrange(q,Rparams4dyn)
% This function derives the dynamic parameters of the Robot

m = Rparams4dyn(1);
L = Rparams4dyn(2);
gama0 = Rparams4dyn(3);
gama1 = Rparams4dyn(4);
gama2 = Rparams4dyn(5) ;
gama3 = Rparams4dyn(6);
k1 = Rparams4dyn(7);
k2 = Rparams4dyn(8);
k3 = Rparams4dyn(9);
I  = Rparams4dyn(10);


E  = 1.38e9;
g0 = 9.81;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q1 = q(1);
q2 = q(2);
q3 = q(3);

%% A*ddq represents the inertia moment of rigid links 1, 2 and 3.
A = zeros(3,3);
A(1,1) = ((1/3)*gama1^3 + (1/3)*gama2^3 + (1/3)*gama3^3 + gama1^2*gama2 + gama1^2*gama3 + gama2^2*gama3 + ...
            (gama1*gama2^2 + 2*gama1*gama2*gama3)*cos(q2) + gama2*gama3^2*cos(q3) + ...
            gama1*gama3^2*cos(q2+q3))*m*L^2;
        
A(2,2) = ((1/3)*gama3^3 + gama2^2*gama3 + gama2*gama3^2*cos(q3))*m*L^2;
A(3,3) = (1/3)*gama3^3*m*L^2;

A(1,2) = ((1/3)*gama2^3 + (1/3)*gama3^3 + gama2^2*gama3 + ((1/2)*gama1*gama2^2 + gama1*gama2*gama3)*cos(q2) + ...
            gama2*gama3^2*cos(q3) + (1/2)*gama1*gama3^2*cos(q2+q3))*m*L^2;
A(2,1) = A(1,2);

A(3,1) = ((1/3)*gama3^3 + (1/2)*gama2*gama3^2*cos(q3) + (1/2)*gama1*gama3^2*cos(q2+q3))*m*L^2;
A(1,3) = A(3,1);

A(3,2) = ((1/3)*gama3^3 +(1/2)*gama2*gama3^2*cos(q3))*m*L^2;
A(2,3) = A(3,2);

%% B*dq^2 indicates the centripetal force caused by the angular velocity dq1, dq2, dq3
%
B = zeros(3,3);
B(1,2) = -(((1/2)*gama1*gama2^2 + gama1*gama2*gama3)*sin(q2) + (1/2)*gama1*gama3^2*sin(q2 +q3)) *m*L^2;
% 错误
B(1,2) = - ( (gama1*gama2^2/2 + gama1 * gama2 * gama3) * sin(q2) + gama1 * gama3^3 * sin(q2 + q3)/2)*m*L^2;
B(1,3) = -((1/2)*gama2*gama3^2*sin(q3) + (1/2)*gama1*gama3^2*sin(q2+q3))*m*L^2;
B(2,1) = (((1/2)*gama1*gama2^2 + gama1*gama2*gama3)*sin(q2) + (1/2)*gama1*gama3^2*sin(q2+q3))*m*L^2;
B(2,3) = -(1/2)*gama2*gama3^2*sin(q3)*m*L^2;
B(3,1) = ((1/2)*gama2*gama3^2*sin(q3) + (1/2)*gama1*gama3^2*sin(q2+q3))*m*L^2;
B(3,2) = (1/2)*gama2*gama3^2*sin(q3)*m*L^2;


%% C*dq(i)*dq(j) describes the coriolis force

C = zeros(3,3);
C(1,1) = -((gama1*gama2^2 + 2*gama1*gama2*gama3)*sin(q2) + gama1*gama3^2*sin(q2+q3))*m*L^2;

C(1,2) = -(gama2*gama3^2*sin(q3)+ gama1*gama3^2*sin(q2+q3))*m*L^2;
C(1,3) = C(1,2);

C(2,2) = -gama2*gama3^2*sin(q3)*m*L^2;
C(2,3) = C(2,2);

C(3,1) = gama2*gama3^2*sin(q3)*m*L^2;

%% D express the deformation energy in torsion springs.
D = zeros(3,3);
D(1,1) = gama1* E*I/L*k1;
D(2,2) = gama2* E*I/L*k2;
D(3,3) = gama3* E*I/L*k3;
end
