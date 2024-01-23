function B_hat = get_B_hat_as_7(q_in1,q_in3)

% q1 = q(1);
% q2 = q(2);
% q3 = q(3);
q1 = q_in1;
q3 = q_in3;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;


B_hat = [3.3*d1^2*cos(q1)^2 + 3.3*d1^2*sin(q1)^2 + 0.59*d2^2*sin(q1)^2 + cos(q3 + 1.6)^2*(0.68*d3^2*cos(q1)^2 + 0.68*d3^2*sin(q1)^2 + 0.078) + 0.98*d1^2 + 2.7*(d1*cos(q1) + 0.5*d3*cos(q3 + 1.6)*cos(q1))^2 + sin(q3 + 1.6)^2*(0.68*d3^2*sin(q1)^2 + 5.4e-4) + 2.7*(d1*sin(q1) + 0.5*d3*cos(q3 + 1.6)*sin(q1))^2 + 0.26,                                                               0, 1.4*d3*sin(q3 + 1.6)*cos(q1)*(d1*sin(q1) + 0.5*d3*cos(q3 + 1.6)*sin(q1)) - 1.0*cos(q1)*(0.68*d3^2*sin(q3 + 1.6)*cos(q1)^2*sin(q1) - 1.0*cos(q3 + 1.6)*sin(q3 + 1.6)*sin(q1)*(0.68*d3^2*cos(q1)^2 + 0.68*d3^2*sin(q1)^2 + 0.078) + cos(q3 + 1.6)*sin(q3 + 1.6)*sin(q1)*(0.68*d3^2*sin(q1)^2 + 5.4e-4)) - 1.4*d3*sin(q3 + 1.6)*sin(q1)*(d1*cos(q1) + 0.5*d3*cos(q3 + 1.6)*cos(q1)) - 1.0*sin(q1)*(cos(q3 + 1.6)*sin(q3 + 1.6)*cos(q1)*(0.68*d3^2*cos(q1)^2 + 0.68*d3^2*sin(q1)^2 + 0.078) - 1.0*cos(q3 + 1.6)*sin(q3 + 1.6)*cos(q1)*(0.68*d3^2*sin(q1)^2 + 5.4e-4) + 0.68*d3^2*sin(q3 + 1.6)*cos(q1)*sin(q1)^2);
                                                                                                                                                                                                                                                                                                              0,                                                             5.1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               1.4*d3*cos(q3 + 1.6)*cos(q1)^2 + 1.4*d3*cos(q3 + 1.6)*sin(q1)^2;
                                                                                          1.4*d3*sin(q3 + 1.6)*cos(q1)*(d1*sin(q1) + 0.5*d3*cos(q3 + 1.6)*sin(q1)) - 1.4*d3*sin(q3 + 1.6)*sin(q1)*(d1*cos(q1) + 0.5*d3*cos(q3 + 1.6)*cos(q1)) - 0.68*d3^2*sin(q3 + 1.6)*cos(q1)*sin(q1)*(cos(q1)^2 + sin(q1)^2), 1.4*d3*cos(q3 + 1.6)*cos(q1)^2 + 1.4*d3*cos(q3 + 1.6)*sin(q1)^2,                                                                                                                                                               sin(q1)*(sin(q1)*(0.68*d3^2*cos(q1)^2 + 0.078)*(cos(q1)^2 + sin(q1)^2) - 0.68*d3^2*cos(q3 + 1.6)*cos(q1)^2*sin(q1)*(cos(q1)^2 + sin(q1)^2)) + 2.7*(0.5*d3*cos(q3 + 1.6)*cos(q1)^2 + 0.5*d3*cos(q3 + 1.6)*sin(q1)^2)^2 + cos(q1)*(cos(q1)*(0.68*d3^2*cos(q1)^2 + 0.078)*(cos(q1)^2 + sin(q1)^2) + 0.68*d3^2*cos(q3 + 1.6)*cos(q1)*sin(q1)^2*(cos(q1)^2 + sin(q1)^2)) + 0.68*d3^2*sin(q3 + 1.6)^2*cos(q1)^2 + 0.68*d3^2*sin(q3 + 1.6)^2*sin(q1)^2];
 

return