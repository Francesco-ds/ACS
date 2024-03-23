function B = get_B_as_7(q_in1,q_in3)

q1 = q_in1;
q3 = q_in3;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;

B = [0.069*sin(q3)^2 + 1.2*d3^2*sin(q3)^2 + 6.3*d1^2 - 2.4*d1*d3*sin(q3) + 0.23,               0,                0;
                                                                         0,             4.6,  -1.2*d3*sin(q3);
                                                                         0, -1.2*d3*sin(q3), 1.2*d3^2 + 0.069];
end