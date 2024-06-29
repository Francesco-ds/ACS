function pose = get_k_as_9(q1_in, q2_in,q3_in)

q1 = q1_in;
q2 = q2_in;
q3 = q3_in;
db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;


% T_BE
T =[-cos(q1)*sin(q3), -cos(q1)*cos(q3),  sin(q1), cos(q1)*(d1 - d3*sin(q3));
-sin(q1)*sin(q3), -cos(q3)*sin(q1), -cos(q1), sin(q1)*(d1 - d3*sin(q3));
         cos(q3),         -sin(q3),        0, d2 + db + q2 + d3*cos(q3);
               0,                0,        0,                         1];

p = T(1:3,4);
phi = [
     atan2(T(2,3), T(1,3));
     atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
     atan2(T(3,2), -T(3,1));
 ];

pose = [p;phi];

end