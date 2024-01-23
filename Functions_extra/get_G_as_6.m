function G = get_G_as_6(q1_in, q2_in,q3_in)
q1 = q1_in;
q2 = q2_in;
q3 = q3_in;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;


G = [0;
    (23544*pi)/3125 + 26487/1250;
    (23544*pi*((d3*cos(q1)^2*cos(q3 + pi/2))/2 + (d3*cos(q3 + pi/2)*sin(q1)^2)/2))/3125];

end