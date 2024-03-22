%analytical_jacobian - from definition
function [analytical_jacobian, analytical_jacobian_val] = analytical_jacobian(tf_matrix_all,q1_val,q2_val,q3_val,d0_val,a1_val,a3_val)
    syms q1 q2 q3 d0 a1 a3 real;

    Pe_b = tf_matrix_all(1:3, 4, end);
    Re_b= tf_matrix_all(1:3, 1:3, end);
    % ZY'Z'' Euler angles
    phi=atan2(Re_b(2,3),Re_b(1,3));
    theta=atan2(sqrt(Re_b(1,3)^2+Re_b(2,3)^2),Re_b(3,3));
    psi=atan2(Re_b(3,2),-Re_b(3,1));

    phi_e=[phi;theta;psi];
    q = [q1; q2; q3];

    Jp = sym(zeros(3, numel(q)));
    for i = 1:numel(q)
        partial_Pe_b_partial_qi = diff(Pe_b, q(i));
        Jp(:, i) = partial_Pe_b_partial_qi;
    end

    Jphi = sym(zeros(3, numel(q)));
    for i = 1:numel(q)
        partial_phi_partial_qi = diff(phi_e, q(i));
        Jphi(:, i) = partial_phi_partial_qi;
    end

    analytical_jacobian = [Jp; Jphi];
    analytical_jacobian = simplify(analytical_jacobian);
    analytical_jacobian_val=double(subs(analytical_jacobian, [q1, q2, q3, d0, a1, a3], [q1_val, q2_val, q3_val, d0_val, a1_val, a3_val]));

end