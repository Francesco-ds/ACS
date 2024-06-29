%set -> even the setting takes long! Hard-coded matrices for operational
%schemes
robot.set_B_operational();
robot.set_C_operational();
robot.set_G_operational();

%display, takes too long! -> for next assignment it's hard-coded to avoid
%compilation problems in simulink

% disp('B_operational: ');
% disp(vpa(simplify(robot.B_operational),2));
% disp('C_operational: ');
% disp(vpa(simplify(robot.C_operational),2));
% disp('G_operational: ');
% disp(vpa(simplify(robot.G_operational),2));