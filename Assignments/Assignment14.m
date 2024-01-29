KD = [80 80 80 10 10 10]';
KP = [50 50 50 50 50 50]';
Md = diag([0.3 0.2 0.1 1 1 1]');
KI = 0;
KF = 5;
Md_inv = inv(Md);
K = diag([0 0 100 0 0 0]);
fd = [0 0 -0.5 0 0 0]';

qi = [0 0 0]';
dqi = [0;0;0];
qf = [0 0.2 0]';
qr = [0 0.1 0]';
st = 0.001;


xi = getK(qi);
xf = getK(qf);
xr = getK(qr);