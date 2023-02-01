%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PRB 3R representation
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% ---------------------  Definition ------------------------------
% 
%   L is the length of the beam.
%   (a,b) is the position of the end-effector(free side of the beam) in the inertial frame after deflection.
%   Phi is the orientation of end-effector
%   F is  force on the beam
%   Mz is and moment applied on the beam.
%   thetaF is the orientation of the force vector.
%   Phi1, Phi2, Phi3 represent the deflection angles of the rigid bodies in PRB 3R representation
%   gamma0, gamma1, gamma2, gamma3, are the ratios of the four link length to the length of the beam 
%       (gamma0 + gamma1 + gamma2 + gamma3 = 0)
%
% ---------------------  PRB parameters ------------------------------
% 
%   k1, k2, k3 are the optimal spring stiffness
%       k1 = 3.51*E*I/L, k2 = 3.51*E*I/L, k3 = 3.51*E*I/L, 
%       where E is the Young’s modulus and E = 1.38*10e9
%       I is the moment of inertia of the beam cross-section around the neutral axis.
% 
% 


clear;

% geo paras
b = 6/1000;         % m width
h = 1/1000;         % m thickness
L = 300/1000;       % m

gama0 = 0.1;
gama1 = 0.35;
gama2 = 0.4;
gama3 = 0.15;

L0 = gama0*L;
L1 = gama1*L;
L2 = gama2*L;
L3 = gama3*L;

%PRBDM_dynamiques;
rhou = 700;         %700kg/m^3 = 7e-07 kg/mm^3 = 0.0007g/mm^3
%m = b*h*L * rhou;   % kg
m = 1;              %假设
m0 = gama0*m;
m1 = gama1*m;
m2 = gama2*m;
m3 = gama3*m;

% PRBDM_parameters dynamiques
E = 1.38e9;
I = 1/12 * h * b^3;
EI = 1.38 * 10^9 * 10^-3 * (6*10^-3)^3/12 ;
J1 = 1/12 * m1 * L1^2;
J2 = 1/12 * m2 * L2^2;
J3 = 1/12 * m3 * L3^2;

k1 = 3.51*E*I/L;
k2 = 2.99*E*I/L;
k3 = 2.58*E*I/L;

g0 = 9.8;  % m/s
W = 0.01;


save('PRBDM_3R_parameters.mat')
