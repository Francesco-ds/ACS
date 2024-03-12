syms d1 d2 d3 db q1(t) q2(t) q3(t) dq1 dq2 dq3

test = [-sin(q1)*(d1 - d3*sin(q3)), 0, -d3*cos(q1)*cos(q3);
 cos(q1)*(d1 - d3*sin(q3)), 0, -d3*cos(q3)*sin(q1);
                         0, 1,         -d3*sin(q3);
                         1, 0,                   0;
                         0, 0,                   0;
                         0, 0,                   1];
 dtest = simplify(diff(test,t))

 dtest = subs(dtest,diff(q1(t), t),dq1);
dtest =   subs(dtest,diff(q2(t), t),dq2);
dtest =    subs(dtest,diff(q3(t), t),dq3);

simplify(dtest)