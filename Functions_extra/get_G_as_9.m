function G = get_G_as_9(q1_in, q2_in,q3_in)

q1 = q1_in;
q2 = q2_in;
q3 = q3_in;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;


G = [  0;
            45;
-12.0*d3*sin(q3)];

end

