function ddq = forw_dyn_val(q,dq,tau)
%ddq = inv(obj.B) * (tau - obj.C*obj.dq - obj.G);
%gives error in compilation time if not defined singularly, keep it like that

q1 = q(1);
q2 = q(2);
q3 = q(3);
dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);
tau1 = tau(1);
tau2 = tau(2);
tau3 = tau(3);

db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;
    

ddq = [ (70.0*(tau1 - 2.0*dq1*dq3*(1.21*cos(q3)*sin(q3)*d3^2 - 1.21*d1*cos(q3)*d3 + 0.069*cos(q3)*sin(q3)))*(7.88e+32*d3^2 - 2.08e+32*d3^2*sin(q3)^2 + 4.54e+31))/(2.19e+32*sin(q3)^2 + 4.26e+33*d3^2*sin(q3)^2 - 1.0e+33*d3^2*sin(q3)^4 + 6.65e+34*d3^4*sin(q3)^2 - 1.76e+34*d3^4*sin(q3)^4 + 2.01e+34*d1^2 + 1.28e+34*d3^2 + 3.49e+35*d1^2*d3^2 - 7.67e+33*d1*d3*sin(q3) - 9.22e+34*d1^2*d3^2*sin(q3)^2 - 1.33e+35*d1*d3^3*sin(q3) + 3.51e+34*d1*d3^3*sin(q3)^3 + 7.38e+32);
        (4.66e-44*(3.45e+32*sin(q3)^2 + 6.03e+33*d3^2*sin(q3)^2 + 3.17e+34*d1^2 - 1.21e+34*d1*d3*sin(q3) + 1.16e+33)*(2.98e+42*tau2 + 6.13e+44*d3^2*sin(q3)^2 + 5.18e+43*d3^2*tau2 - 2.32e+45*d3^2 + 5.18e+43*d3*tau3*sin(q3) + 3.6e+42*d3*dq3^2*cos(q3) + 6.25e+43*d3^3*dq3^2*cos(q3) + 3.58e+42*d3*dq1^2*cos(q3)*sin(q3)^2 + 6.25e+43*d3^3*dq1^2*cos(q3)*sin(q3)^2 - 6.25e+43*d1*d3^2*dq1^2*cos(q3)*sin(q3) - 1.34e+44))/(2.19e+32*sin(q3)^2 + 4.26e+33*d3^2*sin(q3)^2 - 1.0e+33*d3^2*sin(q3)^4 + 6.65e+34*d3^4*sin(q3)^2 - 1.76e+34*d3^4*sin(q3)^4 + 2.01e+34*d1^2 + 1.28e+34*d3^2 + 3.49e+35*d1^2*d3^2 - 7.67e+33*d1*d3*sin(q3) - 9.22e+34*d1^2*d3^2*sin(q3)^2 - 1.33e+35*d1*d3^3*sin(q3) + 3.51e+34*d1*d3^3*sin(q3)^3 + 7.38e+32);
        (50.0*(6.31e+31*sin(q3)^2 + 1.1e+33*d3^2*sin(q3)^2 + 5.79e+33*d1^2 - 2.21e+33*d1*d3*sin(q3) + 2.13e+32)*(0.603*sin(2.0*q3)*d3^2*dq1^2 - 1.21*d1*cos(q3)*d3*dq1^2 + 11.8*sin(q3)*d3 + 0.0345*sin(2.0*q3)*dq1^2 + tau3))/(2.19e+32*sin(q3)^2 + 4.26e+33*d3^2*sin(q3)^2 - 1.0e+33*d3^2*sin(q3)^4 + 6.65e+34*d3^4*sin(q3)^2 - 1.76e+34*d3^4*sin(q3)^4 + 2.01e+34*d1^2 + 1.28e+34*d3^2 + 3.49e+35*d1^2*d3^2 - 7.67e+33*d1*d3*sin(q3) - 9.22e+34*d1^2*d3^2*sin(q3)^2 - 1.33e+35*d1*d3^3*sin(q3) + 3.51e+34*d1*d3^3*sin(q3)^3 + 7.38e+32) + (2.41*d3*sin(q3)*(1.21*d3*cos(q3)*dq3^2 + tau2 - 44.9)*(3.45e+32*sin(q3)^2 + 6.03e+33*d3^2*sin(q3)^2 + 3.17e+34*d1^2 - 1.21e+34*d1*d3*sin(q3) + 1.16e+33))/(2.19e+32*sin(q3)^2 + 4.26e+33*d3^2*sin(q3)^2 - 1.0e+33*d3^2*sin(q3)^4 + 6.65e+34*d3^4*sin(q3)^2 - 1.76e+34*d3^4*sin(q3)^4 + 2.01e+34*d1^2 + 1.28e+34*d3^2 + 3.49e+35*d1^2*d3^2 - 7.67e+33*d1*d3*sin(q3) - 9.22e+34*d1^2*d3^2*sin(q3)^2 - 1.33e+35*d1*d3^3*sin(q3) + 3.51e+34*d1*d3^3*sin(q3)^3 + 7.38e+32)];

   
end