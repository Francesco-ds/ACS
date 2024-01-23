%RNE formulation

zero = zeros(3,1);
syms g real;
g0 = [0;0;-9.81];
% tau = RNE_formulation(robot,robot.q,robot.dq,robot.ddq,g0); %all params
 % G = RNE_formulation(robot,robot.q,zero,zero,-9.81); %dq ddq to 0
% C = RNE_formulation(robot,robot.q,robot.dq,zero,0); %ddq gravity to 0
% 
% 
% 
% 
B = zeros(3,3,'sym');

for i = 3:3
    ei = zeros(3,1);
    ei(i) = 1; % [1 1 1] diagonal
    B(:,i) = RNE_formulation(robot,robot.q,zero,ei,0); %dq = 0

end

% vpa(simplify(C,2),2)
%compare
syms q1 q2 q3 dq1 dq2 dq3 d1 d2 d3 db real
% B_lag = subs(robot.G,[q1 q2 q3 dq1 dq2 dq3 db d1 d2 d3],[-pi/3 -0.24 pi/7 0.7 0.51 0.31 robot.links_lenghts]);
% B_rne = subs(G,[q1 q2 q3 dq1 dq2 dq3 db d1 d2 d3],[-pi/3 -0.24 pi/7 0.7 0.51 0.31 robot.links_lenghts]);
% B_lag = vpa(B_lag,2)
% B_rne = vpa(B_rne,2)
disp('B_robot')
vpa(simplify(robot.B),2)
disp('B_rne')
vpa(simplify(B),2)
