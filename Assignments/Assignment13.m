% ASS 13
KD = [20;20;20;10;10;10];
KP = [50;50;50;50;50;50];
Mt = [0.3 0.2 0.1 1 1 1];
KDt = [1 1 50 1 1 1];
KPt = [10 10 10 10 10 10];


%trajectory
qi = [0 0 0]';
dqi = [0 0 0]';
qf = [0 0.2 0]';
qr = [0 0.1 0]'; 
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

x = []; dx = []; ddx = []; number = size(q_waypoints);

for i = 1: number(2)
    x(:,i) = get_k_as_9(q_waypoints(1,i),q_waypoints(2,i),q_waypoints(3,i));
    dx(:,i) = get_Ja_as_9(q_waypoints(1,i),q_waypoints(2,i),q_waypoints(3,i)) * dq_waypoints(:,i);
    ddx(:,i) = get_derivative_Ja_as_9(q_waypoints(1,i),q_waypoints(2,i),q_waypoints(3,i),dq_waypoints(1,i),dq_waypoints(2,i),dq_waypoints(3,i)) * dq_waypoints(:,i) + get_Ja_as_9(q_waypoints(1,i),q_waypoints(2,i),q_waypoints(3,i)) * dq_waypoints(:,i);
end

xr = get_k_as_9(qr(1),qr(2),qr(3));
xd = timeseries(x',t_waypoints);
dxd = timeseries(dx',t_waypoints);
ddxd = timeseries(ddx',t_waypoints);

open('Admittance_control.slx')

%% smaller K

K = [1 1 10 1 1 1];
sim('Admittance_control.slx')

%% null K
K = [1 1 1 1 1 1];
sim('Admittance_control.slx')

%%
close('Admittance_control.slx',0)