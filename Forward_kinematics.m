function FK = Forward_kinematics(z)
%this function calculated the forward kinematics of the robot


q1 = z(1);
q2 = z(2);
q3 = z(3);
dq1 = z(4);
dq2 = z(5);
dq3 = z(6);
L0 = z(7);
L1 = z(8);
L2 = z(9);
L3 = z(10);

    
X1 = L0 + L1*cos(q1)+L2*cos(q1+q2)+L3*cos(q1+q2+q3);
X2 = L1*sin(q1)+L2*sin(q1+q2)+L3*sin(q1+q2+q3);
X3 = q1 + q2 + q3;
FK = [X1;X2;X3];

end

