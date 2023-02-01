function F = RRR_CTC_task(z)

q1 = z(1);
q2 = z(2);
q3 = z(3);
dq1 = z(4);
dq2 = z(5);
dq3 = z(6);
x = z(7);
y  = z(8);
theta = z(9);
dx =z(10);
dy = z(11);
dtheta = z(12);
xd = z(13); 
yd = z(14);
thetad = z(15);
dxd = z(16);
dyd = z(17);
dthetad = z(18);
ddxd = z(19);
ddyd = z(20);
ddthetad = z(21);
m1 = z(22);
m2 = z(23);
m3 = z(24); 
L1 = z(25);
L2 = z(26);
L3 = z(27);
W = z(28);
kv = z(29);
kp = z(30);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q = [q1;q2;q3];
dq = [dq1;dq2;dq3];
X = [x;y;theta];
dX = [dx;dy;dtheta];
X_d = [xd;yd;thetad];
dX_d = [dxd;dyd;dthetad];
ddX_d = [ddxd;ddyd;ddthetad];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

params =[];

params(1) = m1;
params(2) = m2;
params(3) = m3; 
params(4) = L1;
params(5) = L2;
params(6) = L3;
params(7) = W;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% computed torque controller
% get dynamics of the manipulator
[M,V,G] = PRBDM_3R_Dynamic_Matrices_NewtonEular(q,dq,params);

% coverting the dynamic matrices into task space
% compute jacobian
j = [q1;q2;q3;L1;L2;L3];
J = Jacobi(j);

% compute derivative of jacobian
dJ = [(-1).*dq1.*L1.*cos(q1)+(-1).*(dq1+dq2).* ... 
L2.*cos(q1+q2)+(-1/2).*(dq1+dq2+dq3).* ...  
L3.*cos(q1+q2+q3),(-1).*(dq1+dq2).*L2.* ...  
cos(q1+q2)+(-1/2).*(dq1+dq2+dq3).*L3.* ...  
cos(q1+q2+q3),(-1/2).*(dq1+dq2+dq3) ...  
.*L3.*cos(q1+q2+q3);(-1).*dq1.*L1.*sin(q1)+( ...  
-1).*(dq1+dq2).*L2.*sin(q1+q2)+(-1/2).*( ...  
dq1+dq2+dq3).*L3.*sin(q1+q2+q3),(-1) ...  
.*(dq1+dq2).*L2.*sin(q1+q2)+(-1/2).*( ...  
dq1+dq2+dq3).*L3.*sin(q1+q2+q3),( ...  
-1/2).*(dq1+dq2+dq3).*L3.*sin(q1+q2+ ...  
q3);0,0,0];

M_tilda = (inv(J))'*M*inv(J);
V_tilda = (inv(J))'*(-M*inv(J)*dJ*inv(J)+V*inv(J));
G_tilda = (inv(J))'*G;

KV     =  kv*eye(3,3);
KP     =  kp*eye(3,3);
v       =  ddX_d-KV*(dX-dX_d)-KP*(X-X_d);
F       =  M_tilda*v+V_tilda*dX+G_tilda;


end