
function B_hat = get_B_hat_as_7(q_in1,q_in3)

% q1 = q(1);
% q2 = q(2);
% q3 = q(3);
q1 = q_in1;
q3 = q_in3;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;


B_hat = [0.13*sin(q3)^2 + 1.1*d2^2*sin(q1)^2 + 1.2*d3^2*sin(q1)^2 + 2.4*d3^2*sin(q3)^2 + 12.0*d1^2 - 4.7*d1*d3*sin(q3) - 1.2*d3^2*sin(q1)^2*sin(q3)^2 + 0.4,               0,    -1.2*d3^2*cos(q1)*cos(q3)*sin(q1);
                                                                                                                                                 0,             9.1,                      -2.4*d3*sin(q3);
                                                                                                                 -1.2*d3^2*cos(q1)*cos(q3)*sin(q1), -2.4*d3*sin(q3), 2.4*d3^2 - 1.2*d3^2*sin(q1)^2 + 0.14];


return