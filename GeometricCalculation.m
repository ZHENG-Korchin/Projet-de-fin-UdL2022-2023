function thetad = GeometricCalculation(xd,yd,phi,L0,L1,L2)
%% Geometric model
% syms theta1 theta2 theta3;
% xd = L*(gama0 + gama1*cos(theta1) + gama2*cos(theta1+theta2) + gama3*cos(theta1+theta2+theta3));
% yd = L*(gama1*sin(theta1) + gama2*sin(theta1+theta2) + gama3*sin(theta1+theta2+theta3));


x = xd - L0;
y = yd;
c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2); 
    % if c2 <-1 OR c2>1, solution don't exist
    if (c2 < -1) || (c2 > 1)
        disp('Solutions dont exist');
        warndlg('Solutions dont exist','Warning');
        pause
    end
s2 = sqrt(1 - c2^2);    % s2>0 or s2<0

K1 = L1 +L2*c2;
K2 = L2*s2;


%% inverse
theta2d = atan2(s2,c2);
theta1d = atan2(y,x) - atan2(K2,K1);
theta3d = phi - theta1d - theta2d; 

thetad = [0 theta1d, theta2d, theta3d];

end
