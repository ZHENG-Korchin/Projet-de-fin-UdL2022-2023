function T = RRR_CTC(x)

q1 = x(1);
q2 = x(2);
q3 = x(3);
dq1 = x(4);
dq2 = x(5);
dq3 = x(6);
qd1 = x(7);
qd2  = x(8);
qd3 = x(9);
dqd1 =x(10);
dqd2 = x(11);
dqd3 = x(12);
ddqd1 = x(13); 
ddqd2 = x(14);
ddqd3 = x(15);
m1 = x(16);
m2 = x(17);
m3 = x(18); 
L1 = x(19);
L2 = x(20);
L3 = x(21);
W = x(22);
kv = x(23);
kp = x(24);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

params =[];

params(1) = m1;
params(2) = m2;
params(3) = m3; 
params(4) = L1;
params(5) = L2;
params(6) = L3;
params(7) = W;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% position, velocity and desired position vectors
q = [q1;q2;q3];
dq = [dq1;dq2;dq3];
qd = [qd1;qd2;qd3];
dqd = [dqd1;dqd2;dqd3];
ddqd = [ddqd1;ddqd2;ddqd3];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

% computed torque controller

[M,C,G] = RRR_Dynamic_Matrices(q,dq,params);
KV     =  kv*eye(3,3);
KP     =  kp*eye(3,3);
v       =  ddqd-KV*(dq-dqd)-KP*(q-qd);
T       =  M*v+C*dq+G;



