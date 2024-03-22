classdef MyRobot < handle

    properties

        robot rigidBodyTree
        config struct
        links 
        T struct
        links_lenghts double
        direct_kin sym
        Geo_Jac sym
        Ana_Jac sym
        DH sym
        new_Ja sym
        J_A_valuated
        Ja_derivative
        Ja_inverse
        Tau sym
        B sym
        C sym
        G sym
        B_operational sym
        C_operational sym
        G_operational sym
        q sym
        dq sym
        ddq sym
        kin_ene
        pot_ene
        T_base 
        T_rne
        T_giunto
        T_time_var
        Geo_Jac_time_var
        T_frames
    end

    methods 
        % Initialization of the robot
        function this = MyRobot()
            this.robot = importrobot('RPR_zzy.urdf');
            this.config = homeConfiguration(this.robot); % home configuration as defaukt
            this.links_lenghts = [0.15 0.4 0.3 0.24]; % manual input of links lenghts
            this.q = sym('q', [3 1], 'real'); % sets syms q1 q2 q3
            this.dq = sym('dq', [3 1], 'real'); % sets syms vel
            this.ddq = sym('ddq', [3 1], 'real'); % sets syms acc         
            

        end


    end

    methods


        % show the robot
        function [] = show_robot(this)
            show(this.robot,this.config);
            xlim([-0.5 0.8])
            ylim([-0.5 0.5])
            zlim([0 0.8])
        end
        
        %sets the robot configuration to an input value
        function [] = set_config(this,config)
            this.config(1).JointPosition = config(1);
            this.config(2).JointPosition = config(2);
            this.config(3).JointPosition = config(3);
        end
        
        % gets the transformation matrices
        function [] = set_DH(this)

            syms q1 q2 q3 a alphaa d theta db d1 d2 d3 real
            % order of the DH: a|alfa|d|theta
            DH = [
                0       0      db  0
                d1     0       0       q1
                0       pi/2   d2+q2  0
                d3    0   0       pi/2+q3 
                ];

            % Transformation matrix template
            base=[cos(theta) -sin(theta)*cos(alphaa)  sin(theta)*sin(alphaa)   a*cos(theta);
                sin(theta)  cos(theta)*cos(alphaa)   -cos(theta)*sin(alphaa)  a*sin(theta);
                0           sin(alphaa)              cos(alphaa)              d;
                0           0                       0                       1];

            this.DH = DH; %sets robot attribute

            %Store useful matrices
            T_B0 = subs(base,[a alphaa d theta],[DH(1,1) DH(1,2)   DH(1,3)    DH(1,4)]);
            T_01 = subs(base,[a alphaa d theta],[DH(2,1) DH(2,2)   DH(2,3)    DH(2,4)]);
            T_12 = subs(base,[a alphaa d theta],[DH(3,1) DH(3,2)   DH(3,3)    DH(3,4)]);
            T_2E = subs(base,[a alphaa d theta],[DH(4,1) DH(4,2)   DH(4,3)    DH(4,4)]);
            T_B1 = T_B0 * T_01;
            T_B2 = T_B1 * T_12;
            T_BE = T_B2 * T_2E;
            T_02 = T_01 * T_12;
            T_0E = T_02 * T_2E;
            T_02 = T_01 * T_12;
            T_03 = T_02 * T_2E;

            
            this.T_base = {T_B0 T_B1 T_B2 T_BE}; %transformations from B to Nframe
            this.T_giunto = {T_01 T_02 T_03}; %transformation from first joint to Nframe
            this.T_rne = {T_01 T_12 T_2E}; % transformation joint from frame N to N+1
            this.T_frames = cat(3,T_B0,T_01,T_12,T_2E);%same as before but includes base frame
            this.T = struct("T_B0",T_B0,"T_01",T_01,"T_12",T_12,"T_2E",T_2E,"T_B1",T_B1,"T_B2",T_B2,"T_BE",T_BE,"T_02",T_02,"T_0E",T_0E); %collection of matrices
            
        end
        
        % Same as the upper code but the variables qx are expressed wrt
        % time to get the derivative of matrix Ja later in the code
        function [] = set_T_time_variant(this)

            syms q1(t) q2(t) q3(t) a alphaa d theta db d1 d2 d3 real
            % order of the DH: a alfa d theta
            DH = [
                0       0      db  0
                d1     0       0       q1(t)
                0       pi/2   d2+q2(t)  0
                d3    0   0       pi/2+q3(t) %-pi/2 added
                ];


            base=[cos(theta) -sin(theta)*cos(alphaa)  sin(theta)*sin(alphaa)   a*cos(theta);
                sin(theta)  cos(theta)*cos(alphaa)   -cos(theta)*sin(alphaa)  a*sin(theta);
                0           sin(alphaa)              cos(alphaa)              d;
                0           0                       0                       1];
            DH = DH;
            this.DH = DH;
            T_B0 = subs(base,[a alphaa d theta],[DH(1,1) DH(1,2)   DH(1,3)    DH(1,4)]);
            T_01 = subs(base,[a alphaa d theta],[DH(2,1) DH(2,2)   DH(2,3)    DH(2,4)]);
            T_12 = subs(base,[a alphaa d theta],[DH(3,1) DH(3,2)   DH(3,3)    DH(3,4)]);
            T_2E = subs(base,[a alphaa d theta],[DH(4,1) DH(4,2)   DH(4,3)    DH(4,4)]);
            T_B1 = T_B0 * T_01;
            T_B2 = T_B1 * T_12;
            T_BE = T_B2 * T_2E;
            T_02 = T_01 * T_12;
            T_0E = T_02 * T_2E;

            this.T_base = {T_B0 T_B1 T_B2 T_BE};
            this.T_frames = cat(3,T_B0,T_01,T_12,T_2E);
            this.T_time_var = struct("T_B0",T_B0,"T_01",T_01,"T_12",T_12,"T_2E",T_2E,"T_B1",T_B1,"T_B2",T_B2,"T_BE",T_BE,"T_02",T_02,"T_0E",T_0E);

        end

        % calculates the direct kinematics for the robot
        function [] = set_direct_kinematics(this)
            syms q1 q2 q3 db d1 d2 d3 real
            config_val = [this.config(1).JointPosition this.config(2).JointPosition this.config(3).JointPosition]; %get the configuration values
            T_B_E = subs(this.T.T_BE,[q1 q2 q3 db d1 d2 d3], [config_val this.links_lenghts]); % Use base to EE matrix to get the cartesian position
            %comparison with the toolbox 
            T_B_E = double(T_B_E); 
            robot_T_B_E = getTransform(this.robot,this.config,'ee','base_link'); % gives in output the base to EE matrix
            Manual = T_B_E(:,4);
            Toolbox = robot_T_B_E(:,4);
            disp(table(Manual,Toolbox)); % Check if the result is correct
            this.direct_kin = this.T.T_BE(:,4); %add direct_kin attribute
        end

        %solve the inverse kinematics
        function [] = set_inverse_kinematics(this,config_inverse)

            syms q1 q2 q3 db d1 d2 d3 real
            %see better on pdf report
            equation_x = cos(q1) * (d1 - d3 * sin(q3));
            equation_y = sin(q1) * (d1 - d3 * sin(q3));
            equation_z = db + d2 + q2 + d3 * cos(q3);
            % get the x y z coordinates from direct kinematics with the
            % given configuration in input
            x_coord = subs(equation_x,[q1 q2 q3 db d1 d2 d3], [config_inverse this.links_lenghts]);
            y_coord = subs(equation_y,[q1 q2 q3 db d1 d2 d3], [config_inverse this.links_lenghts]);
            z_coord = subs(equation_z,[q1 q2 q3 db d1 d2 d3], [config_inverse this.links_lenghts]);
            x_coord = double(x_coord);
            y_coord = double(y_coord);
            z_coord = double(z_coord);
            %solve for the joint values, actual inverse kinematics
            q1_sol = real(atan2(y_coord,x_coord));
            q3_solution_equation_sin =asin(-(1/d3) * ((y_coord/sin(q1_sol) - d1)));
            q3_solution_equation_no_distance_sin = subs(q3_solution_equation_sin,[db d1 d2 d3], this.links_lenghts);
            q3_solution_equation_cos = asin(-(1/d3) * ((x_coord/cos(q1_sol) - d1))); % should be always asin????/
            q3_solution_equation_no_distance_cos = subs(q3_solution_equation_cos,[db d1 d2 d3], this.links_lenghts);
            %if q1 is pi/2 then second case is not solvable
            if q1_sol == pi/2
                q3_sol = double(real(subs(q3_solution_equation_no_distance_sin,q1,q1_sol)));
            else
                q3_sol = double(real(subs(q3_solution_equation_no_distance_cos,q1,q1_sol)));
            end
            q2_solution_equation = -(d2 + db + d3 * cos(q3_sol)) + z_coord;
            q2_solution_equation_no_distance = real(subs(q2_solution_equation,[db d1 d2 d3], this.links_lenghts));
            q2_sol = double(real(subs(q2_solution_equation_no_distance,[q1 q3],[q1_sol q3_sol])));
            disp('showing the comparison between estimated and found (left col and right col)')
            Given = config_inverse';
            Manual = [q1_sol q2_sol q3_sol]';
            disp(table(Manual,Given)); %Checking if the result is correct


        

        end

        % Sets the geometric Jacobian of the robot
        function [] = set_geometric_jacobian(this)
            
            T = this.T; % Transformations from base to EE

            % Get the z after the rotations
            z0 = T.T_B0(1:3,1:3) * [0 0 1]';
            z1 = T.T_B1(1:3,1:3) * z0;
            z2 = T.T_B2(1:3,1:3) * z1;

            %get the base EE vectors
            pe = T.T_BE(1:3,4);
            p0 = T.T_B0(1:3,4);
            p2 = T.T_B2(1:3,4);

            %vectors needed for the jacobian calculations
            p_e0 = pe - p0;
            p_e2 = pe - p2;

            %Initialize and fill the geometric jacobian
            Geometric_jacobian =sym(zeros(6, 3));

            %z0 x (pe-p0)
            Geometric_jacobian(1:3,1) = cross(z0,p_e0);
            Geometric_jacobian(4:6,1) = z0;

            %z1
            z1 = T.T_B1(1:3,1:3) * z0;
            Geometric_jacobian(1:3,2) = z1;
            Geometric_jacobian(4:6,3) = [0 0 0]';

            %z2 x (pe-p2)
            z2 = T.T_B2(1:3,1:3) * z1;
            Geometric_jacobian(1:3,3) = cross(z2,p_e2);
            Geometric_jacobian(4:6,3) = z2;
            this.Geo_Jac = Geometric_jacobian;

            % Comparison with toolbox
            syms q1 q2 q3 db d1 d2 d3 real
            joint_val = [this.config(1).JointPosition this.config(2).JointPosition this.config(3).JointPosition];
            Manual = vpa(simplify(subs(this.Geo_Jac,[q1 q2 q3 db d1 d2 d3],[joint_val this.links_lenghts])),2);
            Jg = vpa(geometricJacobian(this.robot,this.config,'ee'),2);
            temp = Jg(1:3,1:3);
            Jg(1:3,1:3) = Jg(4:6,1:3);
            Jg(4:6,1:3) = temp;
            Toolbox = double(vpa(Jg,2));
            disp(table(Manual,Toolbox));

            
        end

        % Same as the previous geometric jacobian but wrt time to have its
        % derivative
        function [] = set_Geo_time_var(this)
            syms q1(t) q2(t) q3(t) real
            T = this.T_time_var;

            z0 = T.T_B0(1:3,1:3) * [0 0 1]';
            z1 = T.T_B1(1:3,1:3) * z0;
            z2 = T.T_B2(1:3,1:3) * z1;
            pe = T.T_BE(1:3,4);
            p0 = T.T_B0(1:3,4);
            p2 = T.T_B2(1:3,4);
            p_e0 = pe - p0;
            p_e2 = pe - p2;
            Geometric_jacobian =sym(zeros(6, 3));
            %z0 x (pe-p0)
            Geometric_jacobian(1:3,1) = cross(z0,p_e0);
            Geometric_jacobian(4:6,1) = z0;
            %z1
            z1 = T.T_B1(1:3,1:3) * z0;
            Geometric_jacobian(1:3,2) = z1;
            Geometric_jacobian(4:6,3) = [0 0 0]';
            %z2 x (pe-p2)
            z2 = T.T_B2(1:3,1:3) * z1;
            Geometric_jacobian(1:3,3) = cross(z2,p_e2);
            Geometric_jacobian(4:6,3) = z2;
            this.Geo_Jac_time_var = Geometric_jacobian;

            
            
        end

        % Same as the normal analytical Jacobian but wrt time to have its
        % derivative
        function Ja_t = set_Ja_time_variant(this)

            syms phi theta q1(t) q2(t) q3(t) db d1 d2 d3 real
            this.set_T_time_variant;
            T = this.T_time_var;
            this.set_Geo_time_var;
            T_matrix = [0 -sin(phi) cos(phi)*sin(theta);
                0   cos(phi)    sin(phi)*sin(theta);
                1   0   cos(theta)];

            TA_matrix = [1 0 0 0 0 0;
                0 1 0 0 0 0
                0 0 1 0 0 0
                0 0 0 0 -sin(phi) cos(phi)*sin(theta)
                0 0 0 0 cos(phi)  sin(phi)*sin(theta)
                0 0 0 1 0   cos(theta)];

            TA_matrix_inverse = inv(TA_matrix);
            Ja_t = TA_matrix_inverse * this.Geo_Jac_time_var;
            


        end

        % Sets the analytical Jacobian
        function [] = set_analitical_jacobian(this)

            syms phi theta q1 q2 q3 db d1 d2 d3 real
            T = this.T; % Base to EE matrix

            % Initialize the T matrix (ZYZ) for Ta 
            % Phi = x(4), Theta = x(5) basically for the EE
            T_matrix = [0 -sin(phi) cos(phi)*sin(theta);
                0   cos(phi)    sin(phi)*sin(theta);
                1   0   cos(theta)];

            % Compose the Ta matrix
            TA_matrix = [eye(3) zeros(3)
                zeros(3) T_matrix];

            TA_matrix_inverse = inv(TA_matrix); % Inverse

            %Get the Analytical jacobian with Ta^-1 * Jg
            this.Ana_Jac = TA_matrix_inverse * this.Geo_Jac;

            %code to solve the evaluated jacobian in a certain
            %configuration
            to_get_angles = T.T_BE(1:3,1:3);
            to_get_angles_values = double(subs(to_get_angles,[q1 q2 q3 db d1 d2 d3],[this.config(1).JointPosition this.config(2).JointPosition this.config(3).JointPosition this.links_lenghts ]));
            euler_angles = rotm2eul(to_get_angles_values,'ZYZ');
            phi_value = euler_angles(1);
            theta_value = euler_angles(2);
            TA_matrix_values = subs(TA_matrix_inverse,[phi theta],[phi_value theta_value]);
            Geometric_jacobian_evaluated = subs(this.Geo_Jac,[q1 q2 q3 db d1 d2 d3],[this.config(1).JointPosition this.config(2).JointPosition this.config(3).JointPosition this.links_lenghts]);
            this.J_A_valuated = TA_matrix_values * Geometric_jacobian_evaluated;

        end

        % Sets the kinetic energy 
        function [] = set_kinetic_energy(this)

            k = simplify( (1/2) * this.dq' * this.B * this.dq); % kinetic energy formulation
            k = vpa(simplify(k),2);
            this.kin_ene = k; %sets the value in the attributes
        end
        
        % Sets the potential energy
        function [] = set_potential_energy(this)

            u = 0; % Starts as 0 (is a scalar)

            % Calculate potential energy for every link and sum them together
            for i = 1:3
                u = u + this.links{i}.link_pot_en;
            end

            u = vpa(simplify(u),2);
            this.pot_ene = u; % Sets the potential energy attribute

        end

        % Sets the model equation (tau explicit)
        function set_lagrangian(this)
            
            this.set_G;
            this.Tau = this.B * this.ddq + this.C * this.dq + this.G;
        end

        % Gets tau
        function set_equation_of_motion(this)

            Tau = this.B * this.ddq + this.C * this.dq + this.G;
            this.Tau = vpa(simplify(Tau),2);
        end

        % Inertia matrix (not used since its calculated in assignment 2)
        function [] = set_B(this)

            B = sym(zeros(3,3));
            for i = 1:3
                link = this.links{i};
                B = B + link.B;

            end
            this.B = B;
        end

        % Coriolis matrix
        function [] = set_C(this)
           % Initialize variables
           C = sym(zeros(3,3));
           B = this.B;
           q = this.q;
           dq = this.dq;
           
           % Gets C matrix through differentiation of B matrix wrt q
           % variables (slides' formulas)
            for i=1:3
                for j=1:3
                    for k=1:3
                        C(i,j) = C(i,j) + 1/2 * ( diff(B(i,j), q(k))  + diff(B(i,k), q(j))- diff(B(j,k), q(i))) * dq(k);
                    end        
                end
            end
            this.C = C;
        end

        % Sets G matrix
        function [] = set_G(this)

            % Initialize variables
            G = sym(zeros(3,1));
            syms g real;
            g = sym('g',[3 1]);
            g0 = [0;0;-9.81]; % Set g0 gravity vector along z

            % G formulation
            for i = 1:3
                g= 0;

                for j = 1:3
                    link_j = this.links{j}; % Get the link i (RPR)
                    Jp = link_j.jacobian; % Get the jacobian of the corresponding link
                    g = g + link_j.mass * g0' * Jp(1:3,i); % Sum the g contribution
                end

                G(i) = g; % Fill the i row of the G matrix
            end

            this.G = vpa(simplify(-G),2); % Assign the G matrix values

        end

        % Assign links to the robot
        function [] = set_links(this,links)

            this.links = links;

            for i = 1:3
                this.links{i}.set_position(i,this,this.q(i),this.dq(i),this.ddq(i)); % Set the values
            end

        end

        %Maybe not needed
        % function Z = get_z_jacobian(this,index)
        %     T = this.T_base{index};
        %     Z = T(1:3,3);
        % end

        % Get the derivative of the analytical hacobian with respect to
        % time
        function dJa_t = set_Ja_derivative(this)
            syms t q1(t) q2(t) q3(t) dq1 dq2 dq3 real
            Ja_t = this.set_Ja_time_variant(); % Assign to robot the derivable Ja
            dJa_t = diff(Ja_t,t); % Differentiate wrt time
            dJa_t = subs(dJa_t,[diff(q1(t) , t) diff(q2(t) , t) diff(q3(t) , t)], [dq1 dq2 dq3]); % Assign dqx when there is a derivative of q
            this.Ja_derivative = vpa(simplify(dJa_t),2); % Simplify the result
        end

        % Gets inertia matrix in the operational space
        function [] = set_B_operational(this)
            Ja = this.Ana_Jac; % Get analytical jacobian
            this.Ja_inverse = vpa(simplify(pinv(simplify(Ja))),2); % It's not squared -> pinv
            this.B_operational = pinv(Ja') *this.B *pinv(Ja); % Formulation             
        end

        % Gets Coriolis matrix in operational space
        function [] = set_C_operational(this)
           
            Ja = this.Ana_Jac;
            dJa = this.set_Ja_derivative(); % Derivative of Ja wrt time
            this.C_operational = pinv(Ja')*this.C*this.dq-this.B_operational*dJa*this.dq; % Formulation
        end
        
        %Gets G matrix in operational space
        function [] = set_G_operational(this)
            this.G_operational = vpa(simplify(simplify(pinv(this.Ana_Jac'))*simplify(this.G)),2); % Formulation
        end

        % Used to get acceleration in the S function -> call manually when
        % B C or G change 
        function ddq = forward_dyn(this)
            tau = sym('tau', [3, 1]);
            %calculate inverse formula for ddq
            ddq = inv(this.B) * (tau - this.C*this.dq - this.G);
            ddq = vpa(simplify(ddq),2);
        end
    end






end