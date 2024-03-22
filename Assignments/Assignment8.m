% one degree of freedom adaptive control law
% For the input manual switch in the model

%parameters
KD = 30;
lambda = 10;
Ktheta = inv(diag([2 1 10]));

%Parameters
I = 0.3;
F = 0.1;
G = 2;
A = 1;
qi = 0;  %needed in s-function
dqi = 0; %needed in s-function
st = 0.001;

open('adaptive_joint_space.slx')
sim('adaptive_joint_space.slx')
