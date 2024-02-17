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


syms q1 q2 q3 dq1 dq2 dq3 d1 d2 d3 db real
disp('B_robot')
vpa(simplify(robot.B),2)
disp('B_rne')
vpa(simplify(B),2)

disp('G_robot')
vpa(simplify(robot.G),2)
disp('G_rne')
vpa(simplify(G),2)
disp('G_robot')

disp('Tau_robot')
C_robot = vpa(simplify(robot.Tau),2)
disp('Tau_rne')
tau = vpa(simplify(tau),2)
vpa(simplify(subs(C_robot,[q1 q2 q3 dq1 dq2 dq3 db d1 d2 d3],[-pi/6 0.2 pi/5 5 -0.2 0.1 robot.links_lenghts])),2)
vpa(simplify(subs(tau,[q1 q2 q3 dq1 dq2 dq3 db d1 d2 d3],[-pi/6 0.2 pi/5 5 -0.2 0.1 robot.links_lenghts])),2)
