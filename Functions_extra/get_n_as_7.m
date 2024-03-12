function n = get_n_as_7(q_1,q_3,dq_1,dq_3)
    %simplify(robot.C * robot.dq + robot.G)
    q1 = q_1;
    q3 = q_3;
    dq1 = dq_1;
    dq3 = dq_3;
    d1 = 0.4;
    d2 = 0.3;
    d3 = 0.24;
    


n = [2.0*dq1*dq3*(0.603*sin(2.0*q3)*d3^2 - 1.21*d1*cos(q3)*d3 + 0.0345*sin(2.0*q3));
                                                                  - 1.21*d3*cos(q3)*dq3^2 + 44.9;
- 11.8*d3*sin(q3) - 1.0*dq1^2*(0.603*sin(2.0*q3)*d3^2 - 1.21*d1*cos(q3)*d3 + 0.0345*sin(2.0*q3))];




end