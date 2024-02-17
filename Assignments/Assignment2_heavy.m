%% build link objects
syms q1 q2 q3 db d1 d2 d3 real
%Cylinder(name,radius,length,com,index)
%Parallelepiped (name,a,b,c,com,index)
link_1 = Cylinder('cyl',0.02,0.35,[-d1/2 0 0],1);
link_2 = Parallelepiped('rec',0.3,0.03,0.03,[0 -d2/2 0],2);
link_3 = Cylinder('cyl',0.02,0.24,[-d3/2 0 0],3); %change to z out

%set mass
link_1.set_mass();
link_2.set_mass();
link_3.set_mass();
link_1.mass = link_1.mass + 2;
link_2.mass = link_2.mass + 1.7;
link_3.mass = link_3.mass + 2.5;
% attach robot to links
link_1.set_robot(robot);
link_2.set_robot(robot);
link_3.set_robot(robot);
robot.set_links({link_1 link_2 link_3});

%sets I
link_1.set_inertia_tensor;
link_2.set_inertia_tensor;
link_3.set_inertia_tensor;

%Jacobian code, written like this for debug, integrate in links
%t_vec = Center of mass pli with respect to pli frame
%pL => wrt sigma 0 -> first joint frame
pL(:,1) = robot.T_giunto{1}(1:3,1:3) * link_1.t_vec + robot.T_giunto{1}(1:3,4);
pL(:,2) = robot.T_giunto{2}(1:3,1:3) * link_2.t_vec + robot.T_giunto{2}(1:3,4);
pL(:,3) = robot.T_giunto{3}(1:3,1:3) * link_3.t_vec + robot.T_giunto{3}(1:3,4);

link_1.t_vec_sigma_zero = pL(:,1);
link_2.t_vec_sigma_zero = pL(:,2);
link_3.t_vec_sigma_zero = pL(:,3);

z0 = [0;0;1];
P0 = [0;0;0];
JPL1_1 = cross(z0,pL(:,1)-P0);
JPL1_2 = [0;0;0];
JPL1_3 = [0;0;0];
JPL1 = [JPL1_1 JPL1_2 JPL1_3];

JPL2_1 = cross(z0, pL(:,2) - P0);
JPL2_2 = robot.T_giunto{1}(1:3,3);
JPL2_3 = [0; 0; 0];
JPL2 = [JPL2_1 JPL2_2 JPL2_3];

JPL3_1 = cross(z0, pL(:,3)-P0);
JPL3_2 = robot.T_giunto{1}(1:3,3);
JPL3_3 = cross(robot.T_giunto{2}(1:3,3), pL(:,3)-robot.T_giunto{2}(1:3,4));
JPL3 = [JPL3_1 JPL3_2 JPL3_3];

JOL1 = [ 0 0 0;
         0 0 0;
         1 0 0];

JOL2 = [ 0 0 0;
         0 0 0;
         1 0 0];

JOL3 = [ 0 0 sin(q1);
         0 0 -cos(q1);
         1 0 0];

link_1.jacobian = [JPL1;JOL1];
link_2.jacobian = [JPL2;JOL2];
link_3.jacobian = [JPL3;JOL3];
H0_1 = robot.T_giunto{1};
H0_2 = robot.T_giunto{2};
H0_3 = robot.T_giunto{3};
IL1_1 = link_1.I_i;
IL2_2 = link_2.I_i;
IL3_3 = link_3.I_i;
 syms m1 m2 m3
B1 = link_1.mass * (JPL1' * JPL1) + (JOL1'*H0_1(1:3,1:3) * IL1_1 * H0_1(1:3,1:3)'*JOL1);
B2 = link_2.mass * (JPL2' * JPL2) + (JOL2'*H0_2(1:3,1:3) * IL2_2 * H0_2(1:3,1:3)'*JOL2);
B3 = link_3.mass * (JPL3' * JPL3) + (JOL3'*H0_3(1:3,1:3) * IL3_3 * H0_3(1:3,1:3)'*JOL3);
% B1 = m1 * (JPL1' * JPL1) + (JOL1'*H0_1(1:3,1:3) * IL1_1 * H0_1(1:3,1:3)'*JOL1);
% B2 = m2 * (JPL2' * JPL2) + (JOL2'*H0_2(1:3,1:3) * IL2_2 * H0_2(1:3,1:3)'*JOL2);
% B3 = m3 * (JPL3' * JPL3) + (JOL3'*H0_3(1:3,1:3) * IL3_3 * H0_3(1:3,1:3)'*JOL3);
B = B1 + B2 + B3;
robot.B = vpa(simplify(B),2);

% errore nel link_3 -> 3:3 non costante
link_1.B = vpa(simplify(B1),2);
link_2.B = vpa(simplify(B2),2);
link_3.B = vpa(simplify(B3),2);


%% kinetic energy
robot.set_kinetic_energy();
disp('Kinetic energy is: ');
disp(robot.kin_ene);


%% potential energy
robot.set_potential_energy;
disp('Potential energy is: ');
disp(robot.pot_ene);