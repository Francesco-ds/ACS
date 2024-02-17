
function B_hat = get_B_hat_as_7(q_in1,q_in3)

% q1 = q(1);
% q2 = q(2);
% q3 = q(3);
q1 = q_in1;
q3 = q_in3;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;


B_hat = [0.14*sin(q3)^2 + 2.5*d3^2*sin(q3)^2 + 12.0*d1^2 - 4.9*d1*d3*sin(q3) + 0.37,               0,               0;
                                                                         0,             8.8, -2.5*d3*sin(q3);
                                                                         0, -2.5*d3*sin(q3), 2.5*d3^2 + 0.14];


return