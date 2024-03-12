%RNE formulation

zero = zeros(3,1);
syms g real;
g0 = [0;0;-9.81];
tau = RNE_formulation(robot,robot.q,robot.dq,robot.ddq,-9.81); %all params
G = RNE_formulation(robot,robot.q,zero,zero,-9.81); %dq ddq to 0
C = RNE_formulation(robot,robot.q,robot.dq,zero,0); %ddq gravity to 0
B = zeros(3,3,'sym');

for i = 1:3
    ei = zeros(3,1);
    ei(i) = 1; % [1 1 1] diagonal
    B(:,i) = RNE_formulation(robot,robot.q,zero,ei,0); %dq = 0

end


syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 d1 d2 d3 db real
disp('B_robot')
vpa(simplify(robot.B),2)
disp('B_rne')
vpa(simplify(B),2)

disp('G_robot')
vpa(simplify(robot.G),2)
disp('G_rne')
vpa(simplify(G),2)
disp('G_robot')

disp('C_robot');
C_robot = vpa(simplify(robot.C*robot.dq),2);
disp('C_rne');
C_rne = vpa(simplify(C),2);
vpa(simplify(subs(robot.Tau,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2)
vpa(simplify(subs(tau,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2)
