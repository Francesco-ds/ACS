% constants 
kd = 15;
kp = 300;
KD = [kd;kd;kd];
KP = [kp;kp;kp];

% TODO: trajectory like for 7
qi = [0 0 0]';
qd = [pi/6 0.1 pi/3]';
dqi = [0 0 0]';
st = 0.001;
d3_d = 0.24;
g_d = [0;
    45;
   -12.0*d3_d*sin(qd(3))];

%0 off 1 on 2 constant equal qd
gravity_on = 1;