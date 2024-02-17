KD = [20;100;20];
KP = [80; 1000; 100];

clear qd dqd ddqd
hat_flag = 1;

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

for i = 1:3


%m[time,q,dq,ddq,dddq,ddddq] = polynomial_cubic_ti_tf(st,ti,tf,qi,qf,dqi,dqf)
    [~,q, dq ,ddq,~,~] = polynomial_cubic_ti_tf(st,ti,tf,qi(i),qf(i),dqi(i),dqf(i));
    q_waypoints(i,:) = q(1,:)';
    dq_waypoints(i,:) = dq(1,:)';
    ddq_waypoints(i,:) = ddq(1,:)';
end

qd.time= t_waypoints;
qd.signals.values = q_waypoints';
qd.signals.dimensions = 3; % num dof, should put dynamic

dqd.time = t_waypoints;
dqd.signals.values = dq_waypoints';
dqd.signals.dimensions = 3;

ddqd.time = t_waypoints;
ddqd.signals.values = ddq_waypoints';
ddqd.signals.dimensions = 3;


