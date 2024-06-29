%% DH table
robot.set_DH();
disp('The DH table of the robot is:');
disp(robot.DH);

%% direct kinematics
%set the config you wanna test
conf_to_test = [-pi/4 +0.23 -pi/3];
robot.set_config(conf_to_test);
robot.set_direct_kinematics();
disp('The robot direct kinematics is: ');
disp(simplify(robot.direct_kin));

%% inverse kinematics
robot.set_inverse_kinematics(conf_to_test);

%% geometrical jacobian
robot.set_geometric_jacobian()
disp('The geometric jacobian of the robot is: ')
disp(simplify(robot.Geo_Jac))

%% analitical jacobian
robot.set_analitical_jacobian()
disp('The analitical jacobian is: ')
disp(simplify(robot.Ana_Jac))

%% simpler jacobian(solved by hand with derivatives instead of Ta^-1)
Ja_temp = robot.Ana_Jac;
Ja_temp(4:6,1:3) = [1 0 0; 0 0 0; 0 0 1];
robot.new_Ja = Ja_temp;
disp('Analitical jacobian solved by hand')
disp(vpa(simplify(robot.new_Ja),2));

%% clean useless variables
clear conf_to_test Ja_temp