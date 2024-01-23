function ddq = forw_dyn_val(q,dq,tau)

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
-(1.0*(1.8e+41*d2^2*dq1^2*sin(2.0*q1) - 2.8e+42*d3^2*tau1 - 6.7e+41*tau1 + 2.0e+41*d3^2*dq3^2*sin(2.0*q1) + 8.3e+41*d3^4*dq3^2*sin(2.0*q1) + 4.6e+40*dq1*dq3*sin(2.0*q3) - 5.8e+42*d3^2*tau1*cos(q1)^2 - 3.1e+42*d3^2*tau1*cos(q3)^2 + 7.4e+41*d2^2*d3^2*dq1^2*sin(2.0*q1) + 5.9e+41*d3^2*dq1*dq3*sin(2.0*q3) + 1.7e+42*d3^4*dq1*dq3*sin(2.0*q3) + 3.5e+42*d3^4*dq3^2*cos(q1)^3*sin(q1) + 4.2e+41*d3^2*dq1*dq3*cos(q3)^3*sin(q3) + 3.7e+42*d3^4*dq1*dq3*cos(q3)^3*sin(q3) - 6.6e+42*d1*d3^3*dq1*dq3*cos(q3) - 5.8e+42*d3^2*tau3*cos(q1)*cos(q3)*sin(q1) + 3.5e+42*d3^4*dq1^2*cos(q1)^3*cos(q3)^2*sin(q1) - 7.4e+42*d1*d3^3*dq1*dq3*cos(q3)^3 + 4.0e+41*d3^2*dq3^2*cos(q1)*sin(q1)*sin(q3) + 1.7e+42*d3^4*dq3^2*cos(q1)*sin(q1)*sin(q3) + 3.1e+42*d2^2*d3^2*dq1^2*cos(q1)^3*sin(q1) - 1.6e+42*d1*d3*dq1*dq3*cos(q3) + 4.0e+41*d3^2*dq1^2*cos(q1)*cos(q3)^2*sin(q1) + 1.7e+42*d3^4*dq1^2*cos(q1)*cos(q3)^2*sin(q1) + 1.9e+42*d3^4*dq1^2*cos(q1)*cos(q3)^4*sin(q1) + 1.9e+42*d3^4*dq3^2*cos(q1)*cos(q3)^2*sin(q1) + 3.5e+42*d3^4*dq3^2*cos(q1)^3*sin(q1)*sin(q3) - 3.5e+42*d3^4*dq1*dq3*cos(q1)^2*cos(q3) + 3.5e+42*d3^4*dq1*dq3*cos(q1)^4*cos(q3) - 1.5e+42*d3^3*tau2*cos(q1)*cos(q3)*sin(q1)*sin(q3) + 3.7e+42*d3^4*dq1*dq3*cos(q1)^2*cos(q3)^3*sin(q3) - 1.4e+43*d1*d3^3*dq1*dq3*cos(q1)^2*cos(q3) + 1.7e+42*d2^2*d3^2*dq1^2*cos(q1)*cos(q3)^2*sin(q1) + 1.6e+42*d3^2*dq1*dq3*cos(q1)^2*cos(q3)*sin(q3) + 1.0e+43*d3^4*dq1*dq3*cos(q1)^2*cos(q3)*sin(q3) + 7.0e+42*d3^4*dq1*dq3*cos(q1)^4*cos(q3)*sin(q3)))/(1.8e+42*d3^2*cos(q1)^2 - 3.6e+41*d2^2*cos(q1)^2 - 4.6e+40*cos(q3)^2 + 3.3e+41*d3^2*cos(q3)^2 + 7.0e+42*d3^4*cos(q1)^2 - 2.1e+41*d3^2*cos(q3)^4 + 2.0e+42*d3^4*cos(q3)^2 - 1.9e+42*d3^4*cos(q3)^4 + 4.3e+42*d1^2 + 3.6e+41*d2^2 + 1.6e+42*d3^2 + 3.3e+42*d3^4 + 1.7e+43*d1^2*d3^2 + 1.5e+42*d2^2*d3^2 - 1.6e+42*d1*d3*sin(q3) + 3.7e+43*d1^2*d3^2*cos(q1)^2 + 1.7e+42*d2^2*d3^2*cos(q1)^2 + 1.9e+43*d1^2*d3^2*cos(q3)^2 - 3.1e+42*d2^2*d3^2*cos(q1)^4 + 1.7e+42*d2^2*d3^2*cos(q3)^2 - 6.6e+42*d1*d3^3*sin(q3) - 8.1e+41*d3^2*cos(q1)^2*cos(q3)^2 - 8.7e+42*d3^4*cos(q1)^2*cos(q3)^2 - 1.9e+42*d3^4*cos(q1)^2*cos(q3)^4 - 7.4e+42*d1*d3^3*(sin(q3) - 1.0*sin(q3)^3) - 1.7e+42*d2^2*d3^2*cos(q1)^2*cos(q3)^2 - 1.4e+43*d1*d3^3*cos(q1)^2*sin(q3) + 2.0e+41);
(3.3e-3*(1.3e+43*tau2 - 3.0e+42*tau2*cos(q3)^2 + 1.4e+44*cos(q3)^2 + 1.1e+45*d2^2*cos(q1)^2 - 5.2e+45*d3^2*cos(q1)^2 - 9.8e+44*d3^2*cos(q3)^2 - 2.1e+46*d3^4*cos(q1)^2 + 6.2e+44*d3^2*cos(q3)^4 - 6.0e+45*d3^4*cos(q3)^2 + 5.5e+45*d3^4*cos(q3)^4 + 2.8e+44*d1^2*tau2 + 2.4e+43*d2^2*tau2 + 1.7e+44*d3^2*tau2 + 4.6e+44*d3^4*tau2 - 1.3e+46*d1^2 - 1.1e+45*d2^2 - 4.8e+45*d3^2 - 9.8e+45*d3^4 - 5.1e+46*d1^2*d3^2 - 4.4e+45*d2^2*d3^2 + 4.8e+45*d1*d3*sin(q3) - 1.1e+47*d1^2*d3^2*cos(q1)^2 - 4.9e+45*d2^2*d3^2*cos(q1)^2 - 5.7e+46*d1^2*d3^2*cos(q3)^2 + 9.3e+45*d2^2*d3^2*cos(q1)^4 - 4.9e+45*d2^2*d3^2*cos(q3)^2 - 6.4e+43*d3^3*dq3^2*cos(q3)^3 - 2.8e+44*d3^5*dq3^2*cos(q3)^3 + 2.3e+44*d3*tau3*sin(q3) + 2.4e+45*d1^2*d3^2*tau2 + 2.1e+44*d2^2*d3^2*tau2 + 1.6e+43*d3*dq3^2*cos(q3) - 5.3e+43*d3*tau3*(sin(q3) - 1.0*sin(q3)^3) + 2.0e+46*d1*d3^3*sin(q3) + 9.2e+44*d3^3*tau3*sin(q3) + 2.4e+45*d3^2*cos(q1)^2*cos(q3)^2 + 2.6e+46*d3^4*cos(q1)^2*cos(q3)^2 + 5.5e+45*d3^4*cos(q1)^2*cos(q3)^4 + 2.2e+46*d1*d3^3*(sin(q3) - 1.0*sin(q3)^3) - 3.7e+42*d3*dq3^2*cos(q3)^3 + 2.0e+44*d3^3*dq3^2*cos(q3) + 5.6e+44*d3^5*dq3^2*cos(q3) - 4.6e+44*d3^3*tau3*(sin(q3) - 1.0*sin(q3)^3) - 2.4e+43*d2^2*tau2*cos(q1)^2 + 1.2e+44*d3^2*tau2*cos(q1)^2 - 5.3e+43*d3^2*tau2*cos(q3)^2 + 4.6e+44*d3^4*tau2*cos(q1)^2 - 2.3e+44*d3^4*tau2*cos(q3)^2 - 1.8e+45*d1*d3^2*tau3 + 3.4e+44*d1^2*d3*dq3^2*cos(q3) + 2.9e+43*d2^2*d3*dq3^2*cos(q3) + 1.8e+45*d1*d3^2*tau3*cos(q3)^2 + 4.9e+45*d2^2*d3^2*cos(q1)^2*cos(q3)^2 - 6.4e+43*d3^3*dq3^2*cos(q1)^2*cos(q3)^3 - 5.6e+44*d3^5*dq3^2*cos(q1)^2*cos(q3)^3 - 2.8e+44*d3^5*dq3^2*cos(q1)^4*cos(q3)^3 + 2.9e+45*d1^2*d3^3*dq3^2*cos(q3) + 2.5e+44*d2^2*d3^3*dq3^2*cos(q3) - 1.1e+44*d1*d3*tau2*sin(q3) + 2.4e+45*d1^2*d3^2*tau2*cos(q1)^2 - 2.1e+44*d2^2*d3^2*tau2*cos(q1)^4 + 4.1e+46*d1*d3^3*cos(q1)^2*sin(q3) - 6.4e+43*d1*d3^2*dq3^2*sin(2.0*q3) - 5.6e+44*d1*d3^4*dq3^2*sin(2.0*q3) - 9.2e+44*d1*d3^3*tau2*sin(q3) + 4.8e+45*d1^2*d3*tau3*sin(q3) + 4.1e+44*d2^2*d3*tau3*sin(q3) + 1.4e+44*d3^3*dq3^2*cos(q1)^2*cos(q3) + 2.8e+44*d3^5*dq3^2*cos(q1)^2*cos(q3) + 2.8e+44*d3^5*dq3^2*cos(q1)^4*cos(q3) - 5.3e+43*d3^2*tau2*cos(q1)^2*cos(q3)^2 - 6.9e+44*d3^4*tau2*cos(q1)^2*cos(q3)^2 - 9.2e+44*d1*d3^3*tau2*cos(q1)^2*sin(q3) - 4.1e+44*d2^2*d3*tau3*cos(q1)^2*sin(q3) - 2.8e+44*d3^5*dq1^2*cos(q1)^2*cos(q3)^3*sin(q3) + 2.8e+44*d3^5*dq1^2*cos(q1)^4*cos(q3)^3*sin(q3) - 2.9e+43*d2^2*d3*dq3^2*cos(q1)^2*cos(q3) + 2.9e+45*d1^2*d3^3*dq3^2*cos(q1)^2*cos(q3) - 2.5e+44*d2^2*d3^3*dq3^2*cos(q1)^4*cos(q3) - 2.8e+44*d3^5*dq3^2*cos(q1)^2*cos(q3)*sin(q3) + 2.8e+44*d3^5*dq3^2*cos(q1)^4*cos(q3)*sin(q3) - 5.6e+44*d1*d3^4*dq1*dq3*sin(2.0*q1) - 4.6e+44*d3^3*tau3*cos(q1)^2*cos(q3)^2*sin(q3) + 4.6e+44*d3^3*tau1*cos(q1)*cos(q3)*sin(q1)*sin(q3) - 1.1e+45*d1*d3^4*dq3^2*cos(q1)^2*cos(q3)*sin(q3) - 5.6e+44*d3^5*dq1*dq3*cos(q1)^3*cos(q3)^2*sin(q1) + 5.6e+44*d3^5*dq1*dq3*cos(q1)^3*cos(q3)^4*sin(q1) + 1.4e+44*d3^3*dq1*dq3*cos(q1)*sin(q1)*sin(q3) + 5.6e+44*d3^5*dq1*dq3*cos(q1)*sin(q1)*sin(q3) - 2.5e+44*d2^2*d3^3*dq1^2*cos(q1)^2*cos(q3)*sin(q3) + 2.5e+44*d2^2*d3^3*dq1^2*cos(q1)^4*cos(q3)*sin(q3) - 6.4e+43*d3^3*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1) + 6.4e+43*d3^3*dq1*dq3*cos(q1)*cos(q3)^4*sin(q1) - 5.6e+44*d3^5*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1) + 5.6e+44*d3^5*dq1*dq3*cos(q1)*cos(q3)^4*sin(q1) - 2.8e+44*d3^5*dq1*dq3*cos(q1)^3*cos(q3)^2*sin(q1)*sin(q3) + 1.1e+45*d1*d3^4*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1) + 2.9e+45*d1^2*d3^3*dq1*dq3*cos(q1)*sin(q1)*sin(q3) + 2.5e+44*d2^2*d3^3*dq1*dq3*cos(q1)*sin(q1)*sin(q3) - 2.5e+44*d2^2*d3^3*dq1*dq3*cos(q1)^3*sin(q1)*sin(q3) - 3.2e+43*d3^3*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1)*sin(q3) - 2.8e+44*d3^5*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1)*sin(q3) + 1.1e+45*d1*d3^4*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1)*sin(q3) - 6.0e+44))/(1.8e+42*d3^2*cos(q1)^2 - 3.6e+41*d2^2*cos(q1)^2 - 4.6e+40*cos(q3)^2 + 3.3e+41*d3^2*cos(q3)^2 + 7.0e+42*d3^4*cos(q1)^2 - 2.1e+41*d3^2*cos(q3)^4 + 2.0e+42*d3^4*cos(q3)^2 - 1.9e+42*d3^4*cos(q3)^4 + 4.3e+42*d1^2 + 3.6e+41*d2^2 + 1.6e+42*d3^2 + 3.3e+42*d3^4 + 1.7e+43*d1^2*d3^2 + 1.5e+42*d2^2*d3^2 - 1.6e+42*d1*d3*sin(q3) + 3.7e+43*d1^2*d3^2*cos(q1)^2 + 1.7e+42*d2^2*d3^2*cos(q1)^2 + 1.9e+43*d1^2*d3^2*cos(q3)^2 - 3.1e+42*d2^2*d3^2*cos(q1)^4 + 1.7e+42*d2^2*d3^2*cos(q3)^2 - 6.6e+42*d1*d3^3*sin(q3) - 8.1e+41*d3^2*cos(q1)^2*cos(q3)^2 - 8.7e+42*d3^4*cos(q1)^2*cos(q3)^2 - 1.9e+42*d3^4*cos(q1)^2*cos(q3)^4 - 7.4e+42*d1*d3^3*(sin(q3) - 1.0*sin(q3)^3) - 1.7e+42*d2^2*d3^2*cos(q1)^2*cos(q3)^2 - 1.4e+43*d1*d3^3*cos(q1)^2*sin(q3) + 2.0e+41);
(2.9e+42*tau3 - 6.7e+41*tau3*cos(q3)^2 + 6.1e+43*d1^2*tau3 + 5.2e+42*d2^2*tau3 + 1.2e+43*d3^2*tau3 + 5.9e+41*d3*tau2*sin(q3) + 4.6e+41*d3^2*dq3^2*sin(2.0*q3) + 1.9e+42*d3^4*dq3^2*sin(2.0*q3) + 1.8e+41*d3*tau2*sin(q3)^3 + 1.5e+42*d3^3*tau2*sin(q3) - 5.2e+42*d2^2*tau3*cos(q1)^2 - 5.8e+42*d3^2*tau3*cos(q3)^2 + 1.5e+42*d3^3*tau2*sin(q3)^3 - 6.1e+42*d1*d3^2*tau2 - 7.4e+42*d1*d3^3*dq3^2*cos(q3) + 9.7e+42*d1^2*d3^2*dq3^2*sin(2.0*q3) + 8.3e+41*d2^2*d3^2*dq3^2*sin(2.0*q3) + 6.1e+42*d1*d3^2*tau2*cos(q3)^2 - 3.5e+42*d3^4*dq1^2*cos(q1)^2*cos(q3)^3 + 3.5e+42*d3^4*dq1^2*cos(q1)^4*cos(q3)^3 + 8.8e+41*d3^2*dq1*dq3*sin(2.0*q1) + 3.5e+42*d3^4*dq1*dq3*sin(2.0*q1) + 7.4e+42*d1*d3^3*dq3^2*cos(q3)^3 - 2.3e+43*d1*d3*tau3*sin(q3) + 1.6e+43*d1^2*d3*tau2*sin(q3) + 1.4e+42*d2^2*d3*tau2*sin(q3) - 3.5e+42*d3^4*dq3^2*cos(q1)^2*cos(q3) + 3.5e+42*d3^4*dq3^2*cos(q1)^4*cos(q3) - 5.8e+42*d3^2*tau3*cos(q1)^2*cos(q3)^2 - 2.1e+41*d3^2*dq3^2*cos(q3)^3*sin(q3) - 1.9e+42*d3^4*dq3^2*cos(q3)^3*sin(q3) - 1.4e+42*d2^2*d3*tau2*cos(q1)^2*sin(q3) + 5.8e+42*d3^2*tau1*cos(q1)*cos(q3)*sin(q1) - 1.9e+42*d3^4*dq3^2*cos(q1)^2*cos(q3)^3*sin(q3) + 1.8e+43*d1^2*d3^2*dq1*dq3*sin(2.0*q1) + 1.6e+42*d2^2*d3^2*dq1*dq3*sin(2.0*q1) - 3.1e+42*d2^2*d3^2*dq1^2*cos(q1)^2*cos(q3) + 3.1e+42*d2^2*d3^2*dq1^2*cos(q1)^4*cos(q3) - 3.5e+42*d3^4*dq3^2*cos(q1)^2*cos(q3)*sin(q3) + 3.5e+42*d3^4*dq3^2*cos(q1)^4*cos(q3)*sin(q3) - 1.5e+42*d3^3*tau2*cos(q1)^2*cos(q3)^2*sin(q3) - 3.5e+42*d3^4*dq1*dq3*cos(q1)^3*cos(q3)^2*sin(q1) - 1.7e+42*d2^2*d3^2*dq3^2*cos(q1)^2*cos(q3)*sin(q3) - 3.1e+42*d2^2*d3^2*dq1*dq3*cos(q1)^3*sin(q1) - 4.0e+41*d3^2*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1) - 3.5e+42*d3^4*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1) - 7.0e+42*d3^4*dq1*dq3*cos(q1)^3*cos(q3)^2*sin(q1)*sin(q3) + 1.4e+43*d1*d3^3*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1) - 8.0e+41*d3^2*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1)*sin(q3) - 7.0e+42*d3^4*dq1*dq3*cos(q1)*cos(q3)^2*sin(q1)*sin(q3) - 1.4e+43*d1*d3^3*dq1*dq3*cos(q1)*sin(q1)*sin(q3))/(1.8e+42*d3^2*cos(q1)^2 - 3.6e+41*d2^2*cos(q1)^2 - 4.6e+40*cos(q3)^2 + 3.3e+41*d3^2*cos(q3)^2 + 7.0e+42*d3^4*cos(q1)^2 - 2.1e+41*d3^2*cos(q3)^4 + 2.0e+42*d3^4*cos(q3)^2 - 1.9e+42*d3^4*cos(q3)^4 + 4.3e+42*d1^2 + 3.6e+41*d2^2 + 1.6e+42*d3^2 + 3.3e+42*d3^4 + 1.7e+43*d1^2*d3^2 + 1.5e+42*d2^2*d3^2 - 1.6e+42*d1*d3*sin(q3) + 3.7e+43*d1^2*d3^2*cos(q1)^2 + 1.7e+42*d2^2*d3^2*cos(q1)^2 + 1.9e+43*d1^2*d3^2*cos(q3)^2 - 3.1e+42*d2^2*d3^2*cos(q1)^4 + 1.7e+42*d2^2*d3^2*cos(q3)^2 - 6.6e+42*d1*d3^3*sin(q3) - 8.1e+41*d3^2*cos(q1)^2*cos(q3)^2 - 8.7e+42*d3^4*cos(q1)^2*cos(q3)^2 - 1.9e+42*d3^4*cos(q1)^2*cos(q3)^4 - 7.4e+42*d1*d3^3*(sin(q3) - 1.0*sin(q3)^3) - 1.7e+42*d2^2*d3^2*cos(q1)^2*cos(q3)^2 - 1.4e+43*d1*d3^3*cos(q1)^2*sin(q3) + 2.0e+41)];


end