clear qd dqd ddqd

% Constants 
KD = [100;100;100];
KP = [100;100;100];

%trajectory
qi = [0 0 0]';
dqi = [0 0 0]';
qf = [pi/6 0.1 -pi/3]';
dqf = [0 0 0]';
ti = 0;
tf = 10;
st = 0.01;

q_waypoints = [];
dq_waypoints = [];
ddq_waypoints = [];
t_waypoints = ti:st:tf;

% Cubic polynomial trajectory to show the overlap between desired and actual
for i = 1:3

% [time,q,dq,ddq,dddq,ddddq] = polynomial_cubic_ti_tf(st,ti,tf,qi,qf,dqi,dqf)
    [~,q, dq ,ddq,~,~] = polynomial_cubic_ti_tf(st,ti,tf,qi(i),qf(i),dqi(i),dqf(i));
    q_waypoints(i,:) = q(1,:)';
    dq_waypoints(i,:) = dq(1,:)';
    ddq_waypoints(i,:) = ddq(1,:)';
end

% Construct the signal (need coupling q,t)
qd.time= t_waypoints;
qd.signals.values = q_waypoints';
qd.signals.dimensions = 3; 

dqd.time = t_waypoints;
dqd.signals.values = dq_waypoints';
dqd.signals.dimensions = 3;

ddqd.time = t_waypoints;
ddqd.signals.values = ddq_waypoints';
ddqd.signals.dimensions = 3;

open('inv_dyn_joint_space.slx')

%% USE HAT MATRICES (The matrices B C G are calculated with an error (different masses))
hat_flag = 1;
sim('inv_dyn_joint_space.slx')

%% USE CORRECT MATRICES
hat_flag = 0;
sim('inv_dyn_joint_space.slx')