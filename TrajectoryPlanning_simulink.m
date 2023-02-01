function TrajQ = TrajectoryPlanning_simulink(X)

% On doit pré-installer <<Robotic toolbox >>
theta00 = 0;
theta10 = X(2);
theta20 = X(3);
theta30 = X(4);

theta0d = 0;
theta1d = X(5);
theta2d = X(6);
theta3d = X(7);

L0 = X(8);
L1 = X(9);
L2 = X(10);
L3 = X(11);

t5 = X(12);
t4 = X(13);
t3 = X(14);
t2 = X(15);
t1  = X(16);

%需要输入多项式拟合的系数
% 
% 
% 
% 
% 
% 


%%
% Établir un modèle de robot (méthode D-H )

%Link  = Link([ theta(i)  d(i)        a(i)      alpha(i)]) % 
RLink0 = Link([   0       0           L0        0       ]);  %
RLink1 = Link([   0       0           L1        0       ]);  %
RLink2 = Link([   0       0           L2        0       ]);  %
RLink3 = Link([   0       0           L3        0       ]);  %
%
RLink0.qlim= 0;

Robot1  = SerialLink ([RLink0 RLink1 RLink2 RLink3 ],'name','Robot1');




%% Planification des trajets
% 
%
step = 100;
%
theta_init = [theta00 theta10 theta20 theta30];
theta_fini = [theta0d theta1d theta2d theta3d];
[qd,dqd,ddqd] = jtraj(theta_init,theta_fini,step);
disp(theta_fini)

%TrajQT = [qd(:,2) qd(:,3) qd(:,4) dqd(:,2) dqd(:,3) dqd(:,4) ddqd(:,2) ddqd(:,3) ddqd(:,4)];
%TrajQ = transpose(TrajQT);

q1d   = qd(:,2)';
q2d   = qd(:,3)';
q3d   = qd(:,4)';
dq1d  = dqd(:,2)';
dq2d  = dqd(:,3)';
dq3d  = dqd(:,4)';
ddq1d = ddqd(:,2)';
ddq2d = ddqd(:,3)';
ddq3d = ddqd(:,4)';

tv = (0:(step-1))/(step-1);
Coefq1 = polyfit(tv,q1d,5);
Coefq2 = polyfit(tv,q2d,5);
Coefq3 = polyfit(tv,q3d,5);
Coefdq1 = polyfit(tv,dq1d,5);
Coefdq2 = polyfit(tv,dq2d,5);
Coefdq3 = polyfit(tv,dq3d,5);
Coefddq1 = polyfit(tv,ddq1d,5);
Coefddq2 = polyfit(tv,ddq2d,5);
Coefddq3 = polyfit(tv,ddq3d,5);

TrajQ(1) = Coefq1(1)*t5 + Coefq1(2)*t4 + Coefq1(3)*t3 + Coefq1(4)*t2 + Coefq1(5)*t1 + Coefq1(6);
TrajQ(2) = Coefq2(1)*t5 + Coefq2(2)*t4 + Coefq2(3)*t3 + Coefq2(4)*t2 + Coefq2(5)*t1 + Coefq1(6);
TrajQ(3) = Coefq3(1)*t5 + Coefq3(2)*t4 + Coefq3(3)*t3 + Coefq3(4)*t2 + Coefq3(5)*t1 + Coefq3(6);
TrajQ(4) = Coefdq1(1)*t5 + Coefdq1(2)*t4 + Coefdq1(3)*t3 + Coefdq1(4)*t2 + Coefdq1(5)*t1 + Coefdq1(6);
TrajQ(5) = Coefdq2(1)*t5 + Coefdq2(2)*t4 + Coefdq2(3)*t3 + Coefdq2(4)*t2 + Coefdq2(5)*t1 + Coefdq2(6);
TrajQ(6) = Coefdq3(1)*t5 + Coefdq3(2)*t4 + Coefdq3(3)*t3 + Coefdq3(4)*t2 + Coefdq3(5)*t1 + Coefdq3(6);
TrajQ(7) = Coefddq1(1)*t5 + Coefddq1(2)*t4 + Coefddq1(3)*t3 + Coefddq1(4)*t2 + Coefddq1(5)*t1 + Coefddq1(6);
TrajQ(8) = Coefddq2(1)*t5 + Coefddq2(2)*t4 + Coefddq2(3)*t3 + Coefddq2(4)*t2 + Coefddq2(5)*t1 + Coefddq2(6);
TrajQ(9) = Coefddq3(1)*t5 + Coefddq3(2)*t4 + Coefddq3(3)*t3 + Coefddq3(4)*t2 + Coefddq3(5)*t1 + Coefddq3(6);





% teach
Robot1.teach;

Robot1.plot(qd)

end

