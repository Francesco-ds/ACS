% 1dof adaptice control law

KD = 30;
lambda = 10;
Ktheta = inv(diag([2 1 10]));

I = 0.3;
F = 0.1;
G = 2;

A = 1;
qi = 0;
dqi = 0;
st = 0.001;
