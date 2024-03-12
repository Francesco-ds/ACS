function Jad = new_getJad(q,xTilde,Td)


q1 = q(1);
q2 = q(2);
q3 = q(3);
db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;

Tde = new_getTde(q, Td);
%slide 29, robotics Kinematics
angles = tform2eul(Tde, 'ZYZ');
phi =   angles(1);
theta = angles(2);

% Rd = Td(1:3, 1:3);
Rd = eul2rotm(xTilde(4:6)');
%correct
Ta = [
[1, 0, 0, 0,         0,                   0]
[0, 1, 0, 0,         0,                   0]
[0, 0, 1, 0,         0,                   0]
[0, 0, 0, 0, -sin(phi), cos(phi)*sin(theta)]
[0, 0, 0, 0,  cos(phi), sin(phi)*sin(theta)]
[0, 0, 0, 1,         0,          cos(theta)]    
];

%GEOMETRIC
J =  [-sin(q1)*(d1 - d3*sin(q3)), 0, -d3*cos(q1)*cos(q3);
 cos(q1)*(d1 - d3*sin(q3)), 0, -d3*cos(q3)*sin(q1);
                         0, 1,         -d3*sin(q3);
                         0, 0,             sin(q1);
                         0, 0,            -cos(q1);
                         1, 0,                   0];
% R matrix slide 14 
R = [Rd' zeros(3,3)
    zeros(3,3) Rd'];

Jad = pinv(Ta)*R*J;

