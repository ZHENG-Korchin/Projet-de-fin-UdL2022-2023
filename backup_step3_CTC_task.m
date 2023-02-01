%the main program


load('step2.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Controller
kv = 40;
kp =20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% simulation parameters

T_f = 10; % simulation interval

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
x = [];
y = [];
theta = [];
dx = [];
dy = [];
dtheta = [];
F1 = [];
F2 = [];
F3 = [];

sim('step3')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Plot the links
figure()

for i = 1:5:length(y)
    
    x0 = [0 0];
    x1 = [L1*cos(q1(i)) L1*sin(q1(i))];
    x2 = x1 + [L2*cos(q1(i)+q2(i)) L2*sin(q1(i)+q2(i))];
    x3 = x2 + [L3*cos(q1(i)+q2(i)+q3(i)) L3*sin(q1(i)+q2(i)+q3(i))];
    clf
    plot([x0(1) x1(1)], [x0(2) x1(2)], 'r-');
    hold on
    plot([x1(1) x2(1)], [x1(2) x2(2)], 'g-');
    plot([x2(1) x3(1)], [x2(2) x3(2)], 'b-');
    
    xlim([-4 4])
    ylim([-4 4])
    axis equal
    drawnow 
end
figure
plot(tout,yd,'r')
hold on
plot(tout,y,'b')
legend('yd','y')
