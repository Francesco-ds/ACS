% Assignment 11
% Study the compliance control. Simulate the two “extreme” cases when the
% end-effector is interacting with the environment (considered as a planar surface)


KD = [50;50;50;10;10;10];
KP = [50;50;50;10;10;10];
qi = [0;0;0];
dqi = [0;0;0];
qf = [0; 0.2; 0];
qr = [0; 0.1; 0];
Ts = 0.1;
xi = new_getK(qi);
xd = new_getK(qf);
xr = new_getK(qr);

open('compliance_control.slx')

%% case K << KP
K=diag([1 1 1 1 1 1]);
sim('compliance_control.slx')

%% case K = KP
K=diag([1 1 50 1 1 1]);
sim('compliance_control.slx')

%% case K >> KP
K=diag([1 1 500 1 1 1]);
sim('compliance_control.slx')

%% 
close_system('compliance_control.slx', 0);