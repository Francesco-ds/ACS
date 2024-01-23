%equation of motion
%already in from ass 2
disp('Matrix B is: ')
disp(vpa(simplify(robot.B),2))

%set C matrix
robot.set_C
disp('Matrix C is: ')
disp(vpa(simplify(robot.C),2))

%set G matrix
robot.set_G
disp('Matrix G is: ')
disp(vpa(simplify(robot.G),2))

%set Tau
robot.set_equation_of_motion()
disp('Tau is: ')
disp(vpa(simplify(robot.Tau),2))
