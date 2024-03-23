function Ja_d = get_derivative_Ja_as_9(q1_in, q2_in,q3_in,dq1_in,dq2_in,dq3_in)



db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;
q1 = q1_in;
q2 = q2_in;
q3 = q3_in;
dq1 = dq1_in;
dq2 = dq2_in;
dq3 = dq3_in;

 Ja_d = [  d3*dq3*cos(q3)*sin(q1) - dq1*cos(q1)*(d1 - d3*sin(q3)), 0, d3*dq1*cos(q3)*sin(q1) + d3*dq3*cos(q1)*sin(q3);
- dq1*sin(q1)*(d1 - d3*sin(q3)) - d3*dq3*cos(q1)*cos(q3), 0, d3*dq3*sin(q1)*sin(q3) - d3*dq1*cos(q1)*cos(q3);
                                                                   0, 0,                                          -d3*dq3*cos(q3);
                                                                   0, 0,                                                           0;
                                                                   0, 0,                                                           0;
                                                                   0, 0,                                                           0];
 
end