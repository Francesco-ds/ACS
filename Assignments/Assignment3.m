% Compute the equation of motion, requires B,C,G matrices

%% Display the B matrix already calculated in assignment 2
disp('Matrix B is: ')
disp(vpa(simplify(robot.B),2))

% Checks for B matrix
disp('Is equal B(2,1) and B(1,2)');
if isequal(robot.B(1,2),robot.B(2,1)) answer = 'Yes';, else answer = 'No';, end 
disp(answer);
disp(' ');
disp('Is equal B(2,3) and B(3,2)'); 
if isequal(robot.B(2,3),robot.B(3,2)) answer = 'Yes';, else answer = 'No';, end 
disp(answer);
disp(' ');
disp('Last element should be free from any variable q')
disp(vpa(simplify(robot.B(3,3)),2))
%% Sets the C matrix
robot.set_C;
disp('Matrix C is: ')
disp(vpa(simplify(robot.C),3)) % Displays the C matrix

%% Sets the G matrix
robot.set_G;
disp('Matrix G is: ')
disp(vpa(simplify(robot.G),2))

%% Sets the equation of motion with Tau explicit
robot.set_equation_of_motion();
disp('Tau is: ')
disp(vpa(simplify(robot.Tau),2))
