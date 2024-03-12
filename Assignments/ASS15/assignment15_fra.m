% Assignement 15
% Implement the Parallel Force/Position Control.


KD = [50;50;20;10;10;10];
KP = [100;100;10;20;20;20];
Md = diag([0.3;0.2;0.1;1;1;1]);
KI = 1.5;
KF = 5;
invMd = inv(Md);


K = diag([1 1 30 0.1 0.1 0.1]);
K = diag([0 0 30 0 0 0]);
fd = [0 0 0.5 0 0 0]';

qi = [0 0 pi/2]';
dqi = [0;0;0];
qf = [pi/2 0.2 pi/2]';
qr = [0 0.1 0];

Ts = 0.001;

xi = new_getK(qi);
xd = new_getK(qf);
xr = new_getK(qr);
