function Ja = get_Ja_as_9(q1_in, q2_in,q3_in)

q1 = q1_in;
q2 = q2_in;
q3 = q3_in;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;


T_BE = [
-cos(q1)*sin(q3), -cos(q1)*cos(q3),  sin(q1); 
-sin(q1)*sin(q3), -cos(q3)*sin(q1), -cos(q1);
         cos(q3),         -sin(q3),        0]; 

euler_angles = rotm2eul(T_BE,'ZYZ');
phi = euler_angles(1);
theta = euler_angles(2);

Ja =[- d1*sin(q1) - d3*cos(q3 + pi/2)*sin(q1), 0,                                                                                                                                    -d3*cos(q1)*sin(q3 + pi/2);
  d1*cos(q1) + d3*cos(q1)*cos(q3 + pi/2), 0,                                                                                                                                    -d3*sin(q1)*sin(q3 + pi/2);
                                       0, 1,                                                                                                     d3*cos(q1)^2*cos(q3 + pi/2) + d3*cos(q3 + pi/2)*sin(q1)^2;
                                       1, 0, (cos(q1)*cos(theta)*sin(phi))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2) - (cos(phi)*cos(theta)*sin(q1))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2);
                                       0, 0,                                                                 - (cos(phi)*cos(q1))/(cos(phi)^2 + sin(phi)^2) - (sin(phi)*sin(q1))/(cos(phi)^2 + sin(phi)^2);
                                       0, 0,                       (cos(phi)*sin(q1))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2) - (cos(q1)*sin(phi))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2)];



end