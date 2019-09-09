function ddq = RealrobotDynamics(q1,q2,dq1,dq2,tau1,tau2)
% Dynamic model of a planar 2DOF manipulator 
% Input:
%   q1,2:   joint position vector
%   dq1,2:  joint velocity vector
%   tau1,2: joint torques
% Output:
%   ddq:    joint acceleration vector

%% Mechanical parameters
% Inertias
I1z = 1.0209; 
I2z = 1.0218;
% Masses
m1 = 5.1368;
m2 = 5.2418;
% Link lengths
L1 = 1.0446; 
L2 = 1.0111;
% CoG
Lg1 = 0.5451; 
Lg2 = 0.5075;
% Gravity
g = 9.81;
% Friction
D = [101.519,      0; 
     0,         103.8566];

%% Variables definition
tau = [tau1; tau2];
s12 = sin(q1+q2);
c12 = cos(q1+q2);
dq = [dq1; dq2];

% Kinematic functions
s1  = sin(q1);
c1  = cos(q1);
s2  = sin(q2);
c2  = cos(q2);

%%%%% Elements of the Inertia Matrix M
M11 = I1z+I2z+Lg1^2*m1+m2*(L1^2+Lg2^2+2*L1*Lg2*c2); 
M12 = I2z+m2*(Lg2^2+L1*Lg2*c2);
M22 = I2z+Lg2^2*m2;
M = [M11, M12; M12, M22];

%%%%% Coriolis and centrifugal elements
C11 = -(L1*dq2*s2*(Lg2*m2)); 
C12 = -(L1*(dq2+dq1)*s2*(Lg2*m2)); 
C21 = m2*L1*Lg2*s2*dq1; 
C22 = 0;
C = [C11 C12; C21 C22];

%%%%% Gravity :
g1 = m1*Lg1*c1+m2*(Lg2*c12 + L1*c1);
g2 =  m2*Lg2*c12;
G = g*[g1; g2];

%% Computation of acceleration
ddq = inv(M) * (tau - C*dq - D*dq - G);
end