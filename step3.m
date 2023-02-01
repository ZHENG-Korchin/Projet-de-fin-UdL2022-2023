clc;
clear;
% 需要预先安装Robotic toolbox 
% On doit pré-installer <<Robotic toolbox >>
load('PRBDM_3R_parameters.mat')

%position of q3
phi_f = pi/3;
x_f = 0.0536;   %(x,y) of actuator
y_f = 0.135;

x_3 = x_f - L3*cos(phi_f);  %(x,y) of q3
y_3 = y_f - L3*sin(phi_f);


F =10;
Mz = 10;

%% 建立机器人模型(改进D-H法)
% Établir un modèle de robot (méthode D-H modifiée)

%L=Link([ theta(i)  d(i)        a(i)      alpha(i)]) % 使用D-H法建立模型  
%           关节角, 连杆偏距,   连杆长度,   连杆转角
RLink0 = Link([   0       0           L0            0       ]);  %定义连杆0
RLink1 = Link([   0       0           L1            0       ]);  %定义连杆1
RLink2 = Link([   0       0           L2            0       ]);  %定义连杆2
RLink3 = Link([   0       0           L3            0       ]);  %定义连杆3
% 定义每个轴的旋转限度 L1.qlim=[-pi/6,pi/6];
RLink0.qlim=0;
% Définir les limites de rotation pour chaque link

Robot1  = SerialLink ([RLink0 RLink1 RLink2 RLink3 ],'name','Robot1');
% 示教模式teach函数
Robot1.teach;


% 观察在规定各个轴旋转条件下，机械臂的形状
% theta   = [pi/2 -2*pi/3 -2*pi/3 0 ];
% Robot1.plot(theta);
%six_robot.display();

%Q=six_robot.ikine(T);

%% 正向运动学 forward kinematic
% theta   = [pi/2 -2*pi/3 -2*pi/3 0 ];
% T04 = fkine(Robot1,theta);

%% 逆向运动学 inverse kinematic
% q=R.ikine(T,          q0,         m,      options)
%           4X4目标矩阵,初始角度,   自由度, 
%           Matrice de destination, angles initiauxm, m indique DOF
% 其中m指明自由度. 当使用3自由度机械手时，m=[1 1 1 0 0 0]; 详情见help SerialLink/ikine
% q0 = [pi/2 0 0 0 ];
%theta_med = Robot1.ikine(T04,q0);


%% 雅克比矩阵 Matrice de Jacobi
%练习使用雅克比矩阵求解函数
% qn = [0 0.7854 3.1416 0 ];
% %jacob0()求解的是将关节速度映射到世界坐标系中的末端执行器空间速度
% J0 = Robot1.jacob0(qn);
% %jacobe()求解的是将关节速度映射到工具坐标系中的末端执行器空间速度
% Jn = Robot1.jacobe(qn);


%% 轨迹规划 Planification des trajets
% 关节空间轨迹规划
%定义轨迹规划初始关节角度（Theta_init）和终止关节角度（Theta_fini）,步数100
Theta_init = [0     0     0     0    ];
Theta_fini = GeometricCalculation(x_3,y_3,phi_f,L0,L1,L2);
step = 100;
%作用时间
t_action = 5;
%tv = (0:(step-1))/(step-1);
tv = linspace(0,t_action,step);
%关节位置, 关节速度, 关节加速度
[qd,dqd,ddqd] = jtraj(Theta_init,Theta_fini,step);

q1d   = qd(:,2)';
q2d   = qd(:,3)';
q3d   = qd(:,4)';
dq1d  = dqd(:,2)';
dq2d  = dqd(:,3)';
dq3d  = dqd(:,4)';
ddq1d = ddqd(:,2)';
ddq2d = ddqd(:,3)';
ddq3d = ddqd(:,4)';


Coefq1 = polyfit(tv,q1d,5);
Coefq2 = polyfit(tv,q2d,5);
Coefq3 = polyfit(tv,q3d,5);
Coefdq1 = polyfit(tv,dq1d,5);
Coefdq2 = polyfit(tv,dq2d,5);
Coefdq3 = polyfit(tv,dq3d,5);
Coefddq1 = polyfit(tv,ddq1d,5);
Coefddq2 = polyfit(tv,ddq2d,5);
Coefddq3 = polyfit(tv,ddq3d,5);

%验算
% f1 = polyval(Coefq1,tv);
% plot(tv,f1);

Robot1.plot(qd)
legend('q_0','q_1','q_2','q_3');

figure(2);
subplot(2,2,1);
i = 1:4;
plot(qd(:,i));
legend( 'q_0','q_1', 'q_2', 'q_3');
grid on;
title('Position des joints');

subplot(2,2,2);
i = 1:4;
plot(dqd(:,i));
legend( 'q''_0','q''_1', 'q''_2', 'q''_3');
grid on;
title('Vitesse des joints');

subplot(2,2,3);
i = 1:4;
plot(ddqd(:,i));
legend('q"_0','q"_1', 'q"_2', 'q"_3');
grid on;
title('vitesse accélérée des joints');

save('step2.mat')
sim('step2_TrajectoryPlanningModel')