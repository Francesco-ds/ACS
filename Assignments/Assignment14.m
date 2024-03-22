% Assingment 14
% Implement the Force Control with Inner Position Loop


KD = [80;80;80;10;10;10];
KP = [50;50;50;50;50;50];
Md = diag([0.3;0.2;0.1;1;1;1]);
KI = 5;
KF = 5;

invMd = inv(Md);


K = diag([0 0 100 0 0 0]);

fd = [0 0 0.75 0 0 0]';

qi = [0 0 0]';
dqi = [0;0;0];
qf = [0 0.2 0]';
qr = [0 0.1 0]';

Ts = 0.001;

xi = new_getK(qi);
xf = new_getK(qf);
xr = new_getK(qr);

open('force_control_inner_loop.slx')
sim('force_control_inner_loop.slx')
%% only proportional
KI = 1;
KF = 5;
sim('force_control_inner_loop.slx')