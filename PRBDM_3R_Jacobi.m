function J = PRBDM_3R_Jacobi(q,Rparams)

q1 = q(1);
q2 = q(2);
q3 = q(3);
gama1 = Rparams(1);
gama2 = Rparams(2);
gama3 = Rparams(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = zeros(3,3);
J(1,1) = -gama1*sin(q1)-gama2*sin(q1+q2)-gama3*sin(q1+q2+q3);
J(1,2) = -gama2*sin(q1+q2)-gama3*sin(q1+q2+q3);
J(1,3) = -gama3*sin(q1+q2+q3);
J(2,1) = gama1*cos(q1)+gama2*cos(q1+q2)+gama3*cos(q1+q2+q3);
J(2,2) = gama2*cos(q1+q2)+gama3*cos(q1+q2+q3);
J(2,3) = gama3*cos(q1+q2+q3);
J(3,1) = 1;
J(3,2) = 1;
J(3,3) = 1;
end

