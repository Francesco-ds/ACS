function n_hat = get_n_hat_as_7(q_1,q_3,dq_1,dq_3)
q1 = q_1;
q3 = q_3;
dq1 = dq_1;
dq3 = dq_3;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;

    


n_hat = [0.55*d2^2*dq1^2*sin(2.0*q1) + 0.59*d3^2*dq3^2*sin(2.0*q1) + 0.13*dq1*dq3*sin(2.0*q3) + 1.2*d3^2*dq1*dq3*sin(2.0*q3) + 1.2*d3^2*dq3^2*cos(q1)*sin(q1)*sin(q3) - 4.7*d1*d3*dq1*dq3*cos(q3) + 1.2*d3^2*dq1^2*cos(q1)*cos(q3)^2*sin(q1) + 2.4*d3^2*dq1*dq3*cos(q1)^2*cos(q3)*sin(q3);
                                                                                                                                                                                                                                                   - 2.4*d3*cos(q3)*dq3^2 + 89.0;
                                                                                                                                                                                                                               - 0.59*dq1*dq3*sin(2.0*q1)*d3^2 - 23.0*sin(q3)*d3];



end