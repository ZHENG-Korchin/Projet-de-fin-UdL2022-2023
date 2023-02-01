function J_T = Jacobi_T(x)

q1 = x(1);
q2 = x(2);
q3 = x(3);
Fx = x(4);
Fy = x(5);
Mz = x(6);
gama1 = x(7);
gama2 = x(8);
gama3 = x(9);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J(1,1) = -gama1*sin(q1)-gama2*sin(q1+q2)-gama3*sin(q1+q2+q3);
J(1,2) = -gama2*sin(q1+q2)-gama3*sin(q1+q2+q3);
J(1,3) = -gama3*sin(q1+q2+q3);
J(2,1) = gama1*cos(q1)+gama2*cos(q1+q2)+gama3*cos(q1+q2+q3);
J(2,2) = gama2*cos(q1+q2)+gama3*cos(q1+q2+q3);
J(2,3) = gama3*cos(q1+q2+q3);
J(3,1) = 1;
J(3,2) = 1;
J(3,3) = 1;

F = [Fx;Fy;Mz];
J_T = J'*F;
end
