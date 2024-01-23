%% build link objects
syms q1 q2 q3 db d1 d2 d3 real
%Cylinder(name,radius,length,com)
link_1 = Cylinder('cyl',0.02,0.35,[-d1/2 0 0],1);
link_2 = Parallelepiped('rec',0.3,0.03,0.03,[0 -d2/2 0],2);
link_3 = Cylinder('cyl',0.02,0.24,[-d3/2 0 0],3);

%set mass
link_1.set_mass();
link_2.set_mass();
link_3.set_mass();
link_1.mass = link_1.mass + 0.4;
link_2.mass = link_2.mass + 0.2;
link_3.mass = link_3.mass + 0.3;
%
link_1.set_robot(robot);
link_2.set_robot(robot);
link_3.set_robot(robot);
robot.set_links({link_1 link_2 link_3});
link_1.set_inertia_matrix;
link_2.set_inertia_matrix;
link_3.set_inertia_matrix;


%% kinetic energy
robot.set_kinetic_energy();
disp('Kinetic energy is: ');
disp(robot.kin_ene);


%% potential energy
robot.set_potential_energy;
disp('Potential energy is: ');
disp(robot.pot_ene);