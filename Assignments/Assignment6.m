% constants 
kd = 5;
kp = 50;
KD = [kd;kd;kd];
KP = [kp;kp;kp];

% TODO: trajectory like for 7
qi = [0 0 0]';
qd = [pi/2 0.1 pi/2]';
dqi = [0 0 0]';
st = 0.001;

gravity_on = 0;