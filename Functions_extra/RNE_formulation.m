function tau = RNE_formulation(robot,q,dq,ddq,g0)
    %disp("sei nel posto giusto")

    
    link_1 = robot.links{1};
    link_2 = robot.links{2};
    link_3 = robot.links{3};
    IL1_1 = link_1.I_i;
    IL2_2 = link_2.I_i;
    IL3_3 = link_3.I_i;
    m1 = link_1.mass;
    m2 = link_2.mass;
    m3 = link_3.mass;
    %syms m1 m2 m3
    z0 = [0 0 1]';
    w0_0 = [0 0 0]';
    d_w0_0 = [0 0 0]';
    dd_p0_0 = [0 0 -g0]';

    
    
    H0_1 = robot.T_rne{1};
    H1_2 = robot.T_rne{2};
    H2_3 = robot.T_rne{3};
    H3_e = [1 0 0 0;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
    % H3_e = [0 0 0 0;
    %         0 0 0 0;
    %         0 0 0 0;
    %         0 0 0 1];
    
    R0_1 = H0_1(1:3,1:3);
    R1_2 = H1_2(1:3,1:3);
    R2_3 = H2_3(1:3,1:3);
    R3_e = H3_e(1:3,1:3);
    
%    distance links
    r1_01 = R0_1' * H0_1(1:3,4);
    r2_12 = R1_2' * H1_2(1:3,4);
    r3_23 = R2_3' * H2_3(1:3,4);
    % r1_01 =H0_1(1:3,4);
    % r2_12 =H1_2(1:3,4);
    % r3_23 =H2_3(1:3,4);
    %^Distances C0M and links

    r1_1c1 = link_1.t_vec;
    r2_2c2 = link_2.t_vec;
    r3_3c3 = link_3.t_vec;

    %first rev joint
    w1_1 = (R0_1)' * w0_0 + R0_1' * dq(1) * z0;
    d_w1_1 = (R0_1)' * d_w0_0 + R0_1' * (ddq(1) * z0 + cross(dq(1) * w0_0,z0));
    dd_p1_1 = (R0_1)' * dd_p0_0 + cross(d_w1_1,r1_01) + cross(w1_1,cross(w1_1,r1_01));
    dd_p1_c1 = dd_p1_1 + cross(d_w1_1,r1_1c1) + cross(w1_1,cross(w1_1,r1_1c1));
    d_w0_m1 = d_w0_0;

    %second prismatic joint
    w2_2 = (R1_2)' * w1_1;
    d_w2_2 = (R1_2)' * d_w1_1;
    dd_p2_2 = (R1_2)' * dd_p1_1 + cross(d_w2_2,r2_12) +cross(w2_2,cross(w2_2,r2_12))+R1_2'*ddq(2)*z0+cross(2*dq(2)*w2_2,R1_2'*z0);
    dd_p2_c2 = dd_p2_2 + cross(d_w2_2,r2_2c2) + cross(w2_2,cross(w2_2,r2_2c2));
    d_w1_m2 = d_w1_1;

    %third rev joint
    w3_3 = (R2_3)' * w2_2 + R2_3' * dq(3) * z0;
    d_w3_3 = (R2_3)' * d_w2_2 + R2_3' * (ddq(3) * z0 + cross(dq(3) * w2_2,z0));
    dd_p3_3 = (R2_3)' * dd_p2_2 + cross(d_w3_3,r3_23) + cross(w3_3,cross(w3_3,r3_23));
    dd_p3_c3 = dd_p3_3 + cross(d_w3_3,r3_3c3) + cross(w3_3,cross(w3_3,r3_3c3));
    d_w2_m3 = d_w2_2;




    % Forces and torques for rne
    f_1 = zeros(3,1,'sym');
    f_2 =  zeros(3,1,'sym');
    f_3 =  zeros(3,1,'sym');
    f_e =  zeros(3,1,'sym');
    mu_1 =  zeros(3,1,'sym');
    mu_2 =  zeros(3,1,'sym');
    mu_3 =  zeros(3,1,'sym');
    mu_e =  zeros(3,1,'sym');
    tau = zeros(3,1,'sym');
    he = [f_e; mu_e];

    % Third rev joint
    f_3 = R3_e * f_e + m3 * dd_p3_c3;
    mu_3 = -cross(f_3, (r3_23+r3_3c3))+R3_e*mu_e+cross(R3_e*f_e,r3_3c3)+IL3_3*d_w3_3+cross(w3_3,IL3_3*w3_3);
    tau(3) = mu_3' * R2_3' * z0;

    % Second prismatic joint
    f_2 = R2_3*f_3+m2*dd_p2_c2;
    mu_2 = -cross(f_2, (r2_12+r2_2c2))+R2_3*mu_3+cross(R2_3*f_3,r2_2c2)+IL2_2*d_w2_2+cross(w2_2,IL2_2*w2_2);
    tau(2) = f_2'*R1_2'*z0;
    
    % First rotative joint
    f_1 = R1_2*f_2+m1*dd_p1_c1;
    mu_1 = -cross(f_1, (r1_01+r1_1c1))+R1_2*mu_2+cross(R1_2*f_2,r1_1c1)+IL1_1*d_w1_1+cross(w1_1,IL1_1*w1_1);
    tau(1) = mu_1'*R0_1'*z0;
    
    %tau = tau;
end
