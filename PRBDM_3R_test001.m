%the main program

clc
clear


xd = 0.0536;
yd = 0.0135;
phi = pi/3;
% RRR Parameters
L1 = 2 ;
L2 = 2;
L3 = 2;
m1 = 0.1;
m2 = 0.1;
m3 = 0.1;
W = 0.01;
g = 9.81;

load('PRBDM_3R_parameters.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Controller
kv = 4;
kp = 2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% simulation parameters

T_f = 5; % simulation interval

AT = 1e-6; % absolute tolerance
RT = 1e-6; % relative tolerance
RF = 4; % Refine factor



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% start simulation

q1 = [];
q2 =[];
q3 = [];
dq1 =[];
dq2 = [];
dq3 =[];
T1 = [];
T2 = [];
T3 = [];

sim('PRBDM_3R_test001mdl')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot results

figure
subplot(3,1,1)
plot(tout,q1*180/pi,'b','linewidth',2)
hold on

plot(tout,qd1*180/pi,'r--','linewidth',2)
legend('q1','qd1')
xlabel('Time(s)')
title('first joint and desired trajectory')


subplot(3,1,2)
plot(tout,q2*180/pi,'b','linewidth',2)
hold on

plot(tout,qd2*180/pi,'r--','linewidth',2)
legend('q2','qd2')
xlabel('Time(s)')
title('second joint and desired trajectory')


subplot(3,1,3)
plot(tout,q3*180/pi,'b','linewidth',2)
hold on

plot(tout,qd3*180/pi,'r--','linewidth',2)
legend('q3','qd3')
xlabel('Time(s)')
title('third joint and desired trajectory')

figure

subplot(3,1,1)
plot(tout,T1,'b','linewidth',2)
xlabel('Time(s)')
ylabel('T1(Nm)')
title('Torque 1')

subplot(3,1,2)
plot(tout,T2,'b','linewidth',2)
xlabel('Time(s)')
ylabel('T2(Nm)')
title('Torque 2')

subplot(3,1,3)
plot(tout,T3,'b','linewidth',2)
xlabel('Time(s)')
ylabel('T3(Nm)')
title('Torque 3')

% errors
e1 = q1 - qd1;
e2 = q2 - qd2;
e3 = q3 - qd3;

figure

subplot(3,1,1)
plot(tout,e1*180/pi,'b','linewidth',2)
xlabel('Time(s)')
ylabel('e1(degrees)')
title('Error 1')

subplot(3,1,2)
plot(tout,e2*180/pi,'b','linewidth',2)
xlabel('Time(s)')
ylabel('e2(degrees)')
title('Error 2')

subplot(3,1,3)
plot(tout,e3*180/pi,'b','linewidth',2)
xlabel('Time(s)')
ylabel('e3(degrees)')
title('Errror 3')

figure
subplot(3,1,1)
plot(tout,dq1*180/pi,'b','linewidth',2)
hold on

plot(tout,dqd1*180/pi,'r--','linewidth',2)
legend('dq1','dqd1')
xlabel('Time(s)')
ylabel('velocity(deg/s)')
title('Joint 1 velocity')

subplot(3,1,2)
plot(tout,dq2*180/pi,'b','linewidth',2)
hold on

plot(tout,dqd2*180/pi,'r--','linewidth',2)
legend('dq2','dqd2')
xlabel('Time(s)')
ylabel('velocity(deg/s)')
title('Joint 2 velocity')

subplot(3,1,3)
plot(tout,dq3*180/pi,'b','linewidth',2)
hold on

plot(tout,dqd3*180/pi,'r--','linewidth',2)
legend('dq3','dqd3')
xlabel('Time(s)')
ylabel('velocity(deg/s)')
title('Joint 3 velocity')


figure
% Plot the links
for i = 1:length(q1)
  x0 = [0 0];
  x1 = [L1*cos(q1(i)) L1*sin(q1(i))];
  x2 = x1 + [L2*cos(q1(i)+q2(i)) L2*sin(q1(i)+q2(i))];
  x3 = x2 + [L3*cos(q1(i)+q2(i)+q3(i)) L3*sin(q1(i)+q2(i)+q3(i))];
 
  cla;
  hold on;
  plot([x0(1) x1(1)], [x0(2) x1(2)], 'r-');
  plot([x1(1) x2(1)], [x1(2) x2(2)], 'g-');
  plot([x2(1) x3(1)], [x2(2) x3(2)], 'b-');
  axis([-0.5 0.5 -0.5 0.5])
  drawnow
end
