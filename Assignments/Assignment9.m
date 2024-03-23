% Gravity compensation in the operational space
% To turn off the gravity check model manual switch

Kd = [70;70;70;10;10;10];
Kp = [1000;1000;1000;50;50;50];

qi = [0 0 0]';
qf = [pi/3 0.4 -pi/6]';
dqi = [0 0 0]';

ti = 0;
tf = 10;
st = 0.001;

% Get cartesian
xi = get_k_as_9(qi(1),qi(2),qi(3));
xd = get_k_as_9(qf(1),qf(2),qf(3));

open('gravity_operational.slx');
sim('gravity_operational.slx');

%% 
close_system('gravity_operational.slx', 0);