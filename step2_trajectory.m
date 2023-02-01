clc;
clear;
close all;
% 需要预先安装Robotic toolbox 
% On doit pré-installer <<Robotic toolbox >>
load('PRBDM_3R_parameters.mat')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% simulation parameters
T_f = 3; % simulation interval
T_step = 0.001; 
AT = 1e-5; % absolute tolerance
RT = 1e-5; % relative tolerance
RF = 4; % Refine factor

%作用时间设置
step = 100;
N = step;
T_action = 1;
t_action = linspace(0,T_action,step);
t_v = linspace(0,T_f,T_f/T_action*step);


% position of the end
phi_f = pi/3;
x_f = 0.0536;   %(x,y) of actuator
y_f = 0.135;

% position of joint3
x_3 = x_f - L3*cos(phi_f);  %(x,y) of joint3
y_3 = y_f - L3*sin(phi_f);
 
% Controller
kv = 40;
kp =20;
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
Theta_fini = GeometricCalculation_simulink([x_f,y_f,phi_f,L0,L1,L2,L3]);
step = 100;
%作用时间T_action
% step = 100;
% T_action = 5;
% t_action = linspace(0,T_action,step);
%关节位置, 关节速度, 关节加速度
[qd,dqd,ddqd] = jtraj(Theta_init,[ 0 Theta_fini],t_action);

q1d   = qd(:,2)';
q2d   = qd(:,3)';
q3d   = qd(:,4)';
dq1d  = dqd(:,2)';
dq2d  = dqd(:,3)';
dq3d  = dqd(:,4)';
ddq1d = ddqd(:,2)';
ddq2d = ddqd(:,3)';
ddq3d = ddqd(:,4)';


Coefq1 = polyfit(t_action,q1d,5);
Coefq2 = polyfit(t_action,q2d,5);
Coefq3 = polyfit(t_action,q3d,5);
Coefdq1 = polyfit(t_action,dq1d,5);
Coefdq2 = polyfit(t_action,dq2d,5);
Coefdq3 = polyfit(t_action,dq3d,5);
Coefddq1 = polyfit(t_action,ddq1d,5);
Coefddq2 = polyfit(t_action,ddq2d,5);
Coefddq3 = polyfit(t_action,ddq3d,5);

%验算
% f1 = polyval(Coefq1,t_action);
% plot(t_action,f1);

Robot1.plot(qd)
legend('q_0','q_1','q_2','q_3');

figure(2);
subplot(1,3,1);
i = 1:4;
plot(t_action, qd(:,i));
legend( 'joint_0','joint_1', 'joint_2', 'joint_3');
grid on;
title('Position des joints');

subplot(1,3,2);
i = 1:4;
plot(t_action, dqd(:,i));
%legend( 'q''_0','q''_1', 'q''_2', 'q''_3');
grid on;
title('Vitesse des joints');

subplot(1,3,3);
i = 1:4;
plot(t_action, ddqd(:,i));
%legend('q"_0','q"_1', 'q"_2', 'q"_3');
grid on;
title('vitesse accélérée des joints');

save('step2.mat')
sim('step2_TrajectoryPlanningModel')