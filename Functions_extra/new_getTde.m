function Tde = new_getTde(q, Td)


q1 = q(1);
q2 = q(2);
q3 = q(3);
db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;


Te =  [-cos(q1)*sin(q3), -cos(q1)*cos(q3),  sin(q1), cos(q1)*(d1 - d3*sin(q3));
-sin(q1)*sin(q3), -cos(q3)*sin(q1), -cos(q1), sin(q1)*(d1 - d3*sin(q3));
         cos(q3),         -sin(q3),        0, d2 + db + q2 + d3*cos(q3);
               0,                0,        0,                         1];
Tde = Td/Te;

end