% 1dof adaptice control law

KD = 40;
lambda = 10;
Ktheta = inv(diag([10 10 10])); % non si puo cambiare il primo termine o esplode?

I = 0.3;
F = 0.1;
G = 2;

A = 1;
qi = 0;
dqi = 0;
st = 0.001;