%% DH table
robot.set_DH();
disp('The DH table of the robot is:');
disp(robot.DH);

%% direct kinematics
%set the config you wanna test
robot.set_config([pi/2 0 pi/2]);
robot.set_direct_kinematics();
disp('The robot direct kinematics is: ');
disp(simplify(robot.direct_kin));

%% inverse kinematics
robot.set_inverse_kinematics([pi/2 0 pi/2]);

%% geometrical jacobian
robot.set_geometric_jacobian()
disp('The geometric jacobian of the robot is: ')
disp(simplify(robot.Geo_Jac))

%% analitical jacobian
robot.set_analitical_jacobian()
disp('The analitical jacobian is: ')
disp(simplify(robot.Ana_Jac))
%syms q1 q2 q3 db d1 d2 d3
%disp(simplify(jacobian(robot.direct_kin,[q1 q2 q3])));