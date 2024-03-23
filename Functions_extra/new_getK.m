function x = new_getK(q)

q1 = q(1);
q2 = q(2);
q3 = q(3);
db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;

T =  [-cos(q1)*sin(q3), -cos(q1)*cos(q3),  sin(q1), cos(q1)*(d1 - d3*sin(q3));
-sin(q1)*sin(q3), -cos(q3)*sin(q1), -cos(q1), sin(q1)*(d1 - d3*sin(q3));
         cos(q3),         -sin(q3),        0, d2 + db + q2 + d3*cos(q3);
               0,                0,        0,                         1];

p = T(1:3,4);

phi = [
    atan2(T(2,3), T(1,3));
    atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
    atan2(T(3,2), -T(3,1));
];

x = [p;phi];

end