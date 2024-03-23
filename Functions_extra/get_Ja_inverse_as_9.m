function Ja_inv = get_Ja_inverse_as_9(q1_in, q2_in,q3_in)

db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;
q1 = q1_in;
q2 = q2_in;
q3 = q3_in;

T_BE = [-cos(q1)*sin(q3), -cos(q1)*cos(q3),  sin(q1); 
-sin(q1)*sin(q3), -cos(q3)*sin(q1), -cos(q1);
         cos(q3),         -sin(q3),        0]; 

euler_angles = rotm2eul(T_BE,'ZYZ');
phi = euler_angles(1);
theta = euler_angles(2);


Ja_inv = [-(sin(q1)*(d1 - d3*sin(q3)))/(d3^2*sin(q3)^2 + d1^2 - 2*d1*d3*sin(q3) + 1), (cos(q1)*(d1 - d3*sin(q3)))/(d3^2*sin(q3)^2 + d1^2 - 2*d1*d3*sin(q3) + 1), 0, 1/(d3^2*sin(q3)^2 + d1^2 - 2*d1*d3*sin(q3) + 1), 0,                                        0;
                      -(d3^2*cos(q1)*cos(q3)*sin(q3))/(d3^2*cos(q3)^2 + 1),                      -(d3^2*cos(q3)*sin(q1)*sin(q3))/(d3^2*cos(q3)^2 + 1), 1,                                               0, 0, (d3*sin(q3))/(d3^2 - d3^2*sin(q3)^2 + 1);
                                -(d3*cos(q1)*cos(q3))/(d3^2*cos(q3)^2 + 1),                                -(d3*cos(q3)*sin(q1))/(d3^2*cos(q3)^2 + 1), 0,                                               0, 0,            1/(d3^2 - d3^2*sin(q3)^2 + 1)];

end