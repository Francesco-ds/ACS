function n_hat = get_n_hat_as_7(q_1,q_3,dq_1,dq_3)
%robot.C*robot.dq+robot.G) 
q1 = q_1;
q3 = q_3;
dq1 = dq_1;
dq3 = dq_3;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;

    


n_hat = [2.0*dq1*dq3*(1.2*sin(2.0*q3)*d3^2 - 2.5*d1*cos(q3)*d3 + 0.07*sin(2.0*q3));
                                            - 2.5*d3*cos(q3)*dq3^2 + 86.0;
                                                         -24.0*d3*sin(q3)];

end