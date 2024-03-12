% case K >> KP env stops robot
KD = [30;30;30;10;10;10];
KP = [30;30;30;10;10;10]; %del robot

qi = [0;0;0];
dqi = [0;0;0];
qf = [0; 0.2; 0];
qr = [0; 0.1; 0];
st = 0.001;
xi = get_k_as_9(qi(1),qi(2),qi(3));
xd = get_k_as_9(qf(1),qf(2),qf(3));
xr = get_k_as_9(qr(1),qr(2),qr(3));

K=diag([1 1 50 1 1 1]); % coefficienti di forza dell'env x y z r1 r2 r3

% case K == KP (env gives same contribute)

% case K << KP