function ddq = forw_dyn_val(q,dq,tau)
%ddq = inv(obj.B) * (tau - obj.C*obj.dq - obj.G);
%gives error in compilation time if not defined singularly, keep it like
%that

q1 = q(1);
q2 = q(2);
q3 = q(3);
dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);
tau1 = tau(1);
tau2 = tau(2);
tau3 = tau(3);

%TODO make also the mass dynamic
db = 0.15;
d1 = 0.4;
d2 = 0.3;
d3 = 0.24;
    

ddq = [
    (70.0*(tau1 - 2.0*dq1*dq3*(1.2*cos(q3)*sin(q3)*d3^2 - 1.2*d1*cos(q3)*d3 + 0.069*cos(q3)*sin(q3)))*(7.9e+32*d3^2 - 2.1e+32*d3^2*sin(q3)^2 + 4.5e+31))/(2.2e+32*sin(q3)^2 + 4.3e+33*d3^2*sin(q3)^2 - 1.0e+33*d3^2*sin(q3)^4 + 6.7e+34*d3^4*sin(q3)^2 - 1.8e+34*d3^4*sin(q3)^4 + 2.0e+34*d1^2 + 1.3e+34*d3^2 + 3.5e+35*d1^2*d3^2 - 7.7e+33*d1*d3*sin(q3) - 9.2e+34*d1^2*d3^2*sin(q3)^2 - 1.3e+35*d1*d3^3*sin(q3) + 3.5e+34*d1*d3^3*sin(q3)^3 + 7.4e+32);
(4.7e-44*(3.5e+32*sin(q3)^2 + 6.0e+33*d3^2*sin(q3)^2 + 3.2e+34*d1^2 - 1.2e+34*d1*d3*sin(q3) + 1.2e+33)*(3.0e+42*tau2 + 6.1e+44*d3^2*sin(q3)^2 + 5.2e+43*d3^2*tau2 - 2.3e+45*d3^2 + 5.2e+43*d3*tau3*sin(q3) + 3.6e+42*d3*dq3^2*cos(q3) + 6.3e+43*d3^3*dq3^2*cos(q3) - 1.3e+44))/(2.2e+32*sin(q3)^2 + 4.3e+33*d3^2*sin(q3)^2 - 1.0e+33*d3^2*sin(q3)^4 + 6.7e+34*d3^4*sin(q3)^2 - 1.8e+34*d3^4*sin(q3)^4 + 2.0e+34*d1^2 + 1.3e+34*d3^2 + 3.5e+35*d1^2*d3^2 - 7.7e+33*d1*d3*sin(q3) - 9.2e+34*d1^2*d3^2*sin(q3)^2 - 1.3e+35*d1*d3^3*sin(q3) + 3.5e+34*d1*d3^3*sin(q3)^3 + 7.4e+32);
(5.3e+73*tau3 + 2.0e+64*d3*sin(q3)^3 + 1.6e+73*tau3*sin(q3)^2 + 3.5e+65*d3^3*sin(q3)^3 + 1.4e+75*d1^2*tau3 + 6.8e+64*d3*sin(q3) + 1.4e+73*d3*tau2*sin(q3) + 1.8e+66*d1^2*d3*sin(q3) + 4.2e+72*d3*tau2*sin(q3)^3 - 7.0e+65*d1*d3^2*sin(q3)^2 + 2.8e+74*d3^2*tau3*sin(q3)^2 + 7.3e+73*d3^3*tau2*sin(q3)^3 - 1.5e+74*d1*d3^2*tau2*sin(q3)^2 - 5.5e+74*d1*d3*tau3*sin(q3) + 1.7e+73*d3^2*dq3^2*cos(q3)*sin(q3) + 3.8e+74*d1^2*d3*tau2*sin(q3) + 5.0e+72*d3^2*dq3^2*cos(q3)*sin(q3)^3 + 8.8e+73*d3^4*dq3^2*cos(q3)*sin(q3)^3 + 4.6e+74*d1^2*d3^2*dq3^2*cos(q3)*sin(q3) - 1.8e+74*d1*d3^3*dq3^2*cos(q3)*sin(q3)^2)/(1.1e+72*sin(q3)^2 + 2.1e+73*d3^2*sin(q3)^2 - 5.0e+72*d3^2*sin(q3)^4 + 3.3e+74*d3^4*sin(q3)^2 - 8.8e+73*d3^4*sin(q3)^4 + 1.0e+74*d1^2 + 6.4e+73*d3^2 + 1.7e+75*d1^2*d3^2 - 3.8e+73*d1*d3*sin(q3) - 4.6e+74*d1^2*d3^2*sin(q3)^2 - 6.7e+74*d1*d3^3*sin(q3) + 1.8e+74*d1*d3^3*sin(q3)^3 + 3.7e+72)];

end