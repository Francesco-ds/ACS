function Jad = get_Jad_as_11(q,x_tilde,T_des)

%slide 14 

q1 = q(1);
q2 = q(2);
q3 = q(3);
db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;

TD_ee = get_T_desired_ee_as_11(q,T_des);

euler_angles = rotm2eul(TD_ee(1:3,(1:3)),'ZYZ');
phi = euler_angles(1);
theta = euler_angles(2);

R_des = eul2rotm(x_tilde(4:6)');

Ta =    [1, 0, 0, 0,         0,                   0;
         0, 1, 0, 0,         0,                   0;
         0, 0, 1, 0,         0,                   0;
         0, 0, 0, 0, -sin(phi), cos(phi)*sin(theta);
         0, 0, 0, 0,  cos(phi), sin(phi)*sin(theta);
         0, 0, 0, 1,         0,          cos(theta);    
         ];
Ja =[- d1*sin(q1) - d3*cos(q3 + pi/2)*sin(q1), 0,                                                                                                                                    -d3*cos(q1)*sin(q3 + pi/2);
  d1*cos(q1) + d3*cos(q1)*cos(q3 + pi/2), 0,                                                                                                                                    -d3*sin(q1)*sin(q3 + pi/2);
                                       0, 1,                                                                                                     d3*cos(q1)^2*cos(q3 + pi/2) + d3*cos(q3 + pi/2)*sin(q1)^2;
                                       1, 0, (cos(q1)*cos(theta)*sin(phi))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2) - (cos(phi)*cos(theta)*sin(q1))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2);
                                       0, 0,                                                                 - (cos(phi)*cos(q1))/(cos(phi)^2 + sin(phi)^2) - (sin(phi)*sin(q1))/(cos(phi)^2 + sin(phi)^2);
                                       0, 0,                       (cos(phi)*sin(q1))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2) - (cos(q1)*sin(phi))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2)]; 

R = [R_des' zeros(3,3)
    zeros(3,3) R_des'];

Jad = pinv(Ta)*R*Ja;