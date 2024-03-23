%RNE formulation

zero = zeros(3,1);
tau = RNE_formulation(robot,robot.q,robot.dq,robot.ddq,-9.81); % All params
G = RNE_formulation(robot,robot.q,zero,zero,-9.81);            % dq ddq to 0
C = RNE_formulation(robot,robot.q,robot.dq,zero,0);            % ddq gravity to 0
% Inertia matrix
B = zeros(3,3,'sym');
for i = 1:3
    ei = zeros(3,1);
    ei(i) = 1; % [1 1 1] diagonal
    B(:,i) = RNE_formulation(robot,robot.q,zero,ei,0); %dq to 0 and ei vector for acc

end


% Check if results are correct
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 d1 d2 d3 db real

% TAU
tau_robot = vpa(simplify(robot.Tau),2);
tau_rne = vpa(simplify(tau),2);
Tau_lag =vpa(simplify(subs(tau_robot,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2);
Tau_rne = vpa(simplify(subs(tau_rne,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2);
disp(table(Tau_lag,Tau_rne))

% C
c_robot = vpa(simplify(robot.C*robot.dq),2);
c_rne = vpa(simplify(C),2);
C_lag =vpa(simplify(subs(c_robot,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2);
C_rne = vpa(simplify(subs(c_rne,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2);
disp(table(C_lag,C_rne))

% G
g_robot = vpa(simplify(robot.G),2);
g_rne = vpa(simplify(G),2);
G_lag =vpa(simplify(subs(g_robot,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2);
G_rne = vpa(simplify(subs(g_rne,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2);
disp(table(G_lag,G_rne))

% B
b_robot = vpa(simplify(robot.B),2);
b_rne = vpa(simplify(B),2);
B_lag =vpa(simplify(subs(b_robot,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2);
B_rne = vpa(simplify(subs(b_rne,[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 db d1 d2 d3 ],[pi/6 -0.15 -pi/3 9 14 -7 -4 2.5 5 robot.links_lenghts])),2);
disp(table(B_lag,B_rne))