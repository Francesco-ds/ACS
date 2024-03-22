% constants 
kd = 20;
kp = 250;
KD = [kd;kd;kd];
KP = [kp;kp;kp];

% Better to leave single point as desired -> important is at infinity
qi = [0 0 0]';
qd = [pi/6 0.1 pi/3]';
dqi = [0 0 0]';
st = 0.001;
d3_d = 0.24;
g_d = [0;
    45;
   -12.0*d3_d*sin(qd(3))];


% 0 There is no gravity control
% 1 There is the realtime gravity control 
% 2 There is only a constant gravity control (final position)
%open('gravity_comp_joint_space.slx')



%% Gravity off no noise
gravity_on = 0;
noise = 0;
sim('gravity_comp_joint_space.slx')

%% Gravity on no noise
gravity_on = 1;
noise = 0;
sim('gravity_comp_joint_space.slx')

%% Gravity desired no noise
gravity_on = 2;
noise = 0;
sim('gravity_comp_joint_space.slx')

%% Gravity off yes noise
gravity_on = 0;
noise = 1;
sim('gravity_comp_joint_space.slx')

%% Gravity on yes noise
gravity_on = 1;
noise = 1;
sim('gravity_comp_joint_space.slx')

%% Gravity desired yes noise
gravity_on = 2;
noise = 1;
sim('gravity_comp_joint_space.slx')

%%
close_system('gravity_comp_joint_space.slx', 0);