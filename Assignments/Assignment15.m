% Assignement 15rol.

KD = [50;50;30;10;10;10];
KP = [100;100;50;20;20;20];
Md = diag([0.3;0.2;0.1;1;1;1]);
KI = 10;
KF = 5;

Md_inv = inv(Md);


K = diag([1 1 10 1 1 1]);
fd = [0 0 -0.1 0 0 0]';

qi = [0 0 0]';
dqi = [0;0;0];
qf = [0 0.2 0]';
qr = [0 0.1 0];

st = 0.001;
xi = get_k_as_9(qi(1),qi(2),qi(3));
xd = get_k_as_9(qf(1),qf(2),qf(3));
xr = get_k_as_9(qr(1),qr(2),qr(3));