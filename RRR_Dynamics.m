function dx = RRR_Dynamics(x)

% Joint positions
q1 = x(1);
q2 = x(2);
q3 = x(3);

% Joint velocities
dq1 = x(4);
dq2 = x(5);
dq3 = x(6);

% Joint Torques
T1 = x(7);
T2 = x(8);
T3 = x(9);

% Robot Parameters

m1 = x(10);
m2 = x(11);
m3 = x(12);
L1 = x(13);
L2 = x(14);
L3 = x(15);
W = x(16);
g0 = 9.81;

% Joint Position, Velocity and Torque Vectors

 q = [ q1;q2;q3];
dq = [dq1;dq2;dq3];
 T = [ T1;T2;T3];
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 
robot_params =[];
robot_params(1) = m1 ;
robot_params(2) = m2 ;
robot_params(3) = m3 ;
robot_params(4) = L1;
robot_params(5) = L2;
robot_params(6) = L3;
robot_params(7) = W;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Dynamics of planar 3-link robot arm
[M,C,G] = RRR_Dynamic_Matrices(q,dq,robot_params);
 
f  = [dq;         -inv(M)*(C*dq+G)];
G  = [zeros(3,3);  inv(M)];


dx = f+G*T;
