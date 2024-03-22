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

%% Check for B and C matrix:
% dB - 2C must be a skew-symmetric matrix
syms q1 q2 q3 dq1 dq2 dq3 db d1 d2 d3 real

% Manually computed, it's correct
B_deriv = [0.003*dq3*(400.0*sin(2.0*q3)*d3^2 - 800.0*d1*cos(q3)*d3 + 23.0*sin(2.0*q3)) , 0 ,0;
    0, 0, -1.2*d3*cos(q3)*dq3;
    0,-1.2*d3*cos(q3)*dq3,0 ];

Matrix_that_should_be_skew_sym = B_deriv - 2*robot.C;
Matrix_that_should_be_skew_sym = vpa(simplify(Matrix_that_should_be_skew_sym),3);

% Matrix_23 = Matrix_that_should_be_skew_sym(2,3)
% Matrix_23 = 


%% Sets the G matrix
robot.set_G;
disp('Matrix G is: ')
disp(vpa(simplify(robot.G),2))

%% Sets the equation of motion with Tau explicit
robot.set_equation_of_motion();
disp('Tau is: ')
disp(vpa(simplify(robot.Tau),2))
