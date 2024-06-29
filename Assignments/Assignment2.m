syms q1 q2 q3 db d1 d2 d3 real

% Constructor formulation
    %Cylinder(name,radius,length,com,index)
    %Parallelepiped (name,a,b,c,com,index)
link_1 = Cylinder('cyl',0.02,0.35,[-d1/2 0 0],1);
link_2 = Parallelepiped('rec',0.3,0.03,0.03,[0 -d2/2 0],2);
link_3 = Cylinder('cyl',0.02,0.24,[-d3/2 0 0],3);

% Sets mass for each link
link_1.set_mass();
link_2.set_mass();
link_3.set_mass();

% Attach robot to each link
link_1.set_robot(robot);
link_2.set_robot(robot);
link_3.set_robot(robot);

% Attach the links to the robot
robot.set_links({link_1 link_2 link_3});

% Sets inertia tensor for each link
link_1.set_inertia_tensor;
link_2.set_inertia_tensor;
link_3.set_inertia_tensor;

%Jacobian code needed for inertia , see slides or report
%t_vec = Center of mass pli with respect to pli frame
%pL => wrt sigma 0 -> first joint frame
pL(:,1) = robot.T_giunto{1}(1:3,1:3) * link_1.t_vec + robot.T_giunto{1}(1:3,4);
pL(:,2) = robot.T_giunto{2}(1:3,1:3) * link_2.t_vec + robot.T_giunto{2}(1:3,4);
pL(:,3) = robot.T_giunto{3}(1:3,1:3) * link_3.t_vec + robot.T_giunto{3}(1:3,4);

% Assign the new values
link_1.t_vec_sigma_zero = pL(:,1);
link_2.t_vec_sigma_zero = pL(:,2);
link_3.t_vec_sigma_zero = pL(:,3);

% z0 and P0 initialization
z0 = [0;0;1];
P0 = [0;0;0];

% First link jacobian
JPL1_1 = cross(z0,pL(:,1)-P0);
JPL1_2 = [0;0;0];
JPL1_3 = [0;0;0];
JPL1 = [JPL1_1 JPL1_2 JPL1_3];

% Second link jacobian
JPL2_1 = cross(z0, pL(:,2) - P0);
JPL2_2 = robot.T_giunto{1}(1:3,3);
JPL2_3 = [0; 0; 0];
JPL2 = [JPL2_1 JPL2_2 JPL2_3];

% Third link jacobian
JPL3_1 = cross(z0, pL(:,3)-P0);
JPL3_2 = robot.T_giunto{1}(1:3,3);
JPL3_3 = cross(robot.T_giunto{2}(1:3,3), pL(:,3)-robot.T_giunto{2}(1:3,4));
JPL3 = [JPL3_1 JPL3_2 JPL3_3];

% First link jacobian
JOL1 = [ 0 0 0;
         0 0 0;
         1 0 0];

% Second link jacobian
JOL2 = [ 0 0 0;
         0 0 0;
         1 0 0];

% Third link jacobian
JOL3 = [ 0 0 sin(q1);
         0 0 -cos(q1);
         1 0 0];

% Compose and assign the jacobian to each link
link_1.jacobian = [JPL1;JOL1];
link_2.jacobian = [JPL2;JOL2];
link_3.jacobian = [JPL3;JOL3];

% Get the transformation matrix from first joint frame 
H0_1 = robot.T_giunto{1};
H0_2 = robot.T_giunto{2};
H0_3 = robot.T_giunto{3}; % 3 is actually EE

%Get inertia for each link
IL1_1 = link_1.I_i;
IL2_2 = link_2.I_i;
IL3_3 = link_3.I_i;

% Calculate the single B matrix for each joint
syms m1 m2 m3 real
B1 = link_1.mass * (JPL1' * JPL1) + (JOL1'*H0_1(1:3,1:3) * IL1_1 * H0_1(1:3,1:3)'*JOL1);
B2 = link_2.mass * (JPL2' * JPL2) + (JOL2'*H0_2(1:3,1:3) * IL2_2 * H0_2(1:3,1:3)'*JOL2);
B3 = link_3.mass * (JPL3' * JPL3) + (JOL3'*H0_3(1:3,1:3) * IL3_3 * H0_3(1:3,1:3)'*JOL3);

% Get the robot B matrix as sum of each link contribution
B = B1 + B2 + B3;
robot.B = vpa(simplify(B),2);

% Display if needed (should be symmetric and bottom right element should be a constant)
link_1.B = vpa(simplify(B1),2);
link_2.B = vpa(simplify(B2),2);
link_3.B = vpa(simplify(B3),2);


%% kinetic energy
robot.set_kinetic_energy();
disp('Kinetic energy is: ');
disp(vpa(simplify(robot.kin_ene),2));

%% potential energy
robot.set_potential_energy;
disp('Potential energy is: ');
disp(vpa(simplify(robot.pot_ene),2));