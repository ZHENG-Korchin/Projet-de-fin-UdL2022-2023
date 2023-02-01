function J = RRR_Jacobi(q1,q2,q3,L1,L2,L3)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J(1,1) = -L1*sin(q1)-L2*sin(q1+q2)-L3/2*sin(q1+q2+q3);
J(1,2) = -L2*sin(q1+q2)-L3/2*sin(q1+q2+q3);
J(1,3) = -L3/2*sin(q1+q2+q3);
J(2,1) = L1*cos(q1)+L2*cos(q1+q2)+L3/2*cos(q1+q2+q3);
J(2,2) = L2*cos(q1+q2)+L3/2*cos(q1+q2+q3);
J(2,3) = L3/2*cos(q1+q2+q3);
J(3,1) = 1;
J(3,2) = 1;
J(3,3) = 1;
end

