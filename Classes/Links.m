classdef Links < handle

    properties 
        density = 8000 %used for mass
        mass double % mass of link
        mass_sym sym
        index double
        robot 
        t_vec % dist com wrt to frame i
        t_vec_sigma_zero % com wrt sigma0
        com_base
        prev_joint_basef
        T_0_i sym % transformation from base
        jacobian sym % partial jacobian wrt li
        I_i sym % inertia tenso wrt frame i
        I_aug sym
        I_0_i sym%inertia tensor wrt frame 0 or base
        B sym % inertia matrix
        K sym % kinetic energy
        pot_energy_link sym % potential energy
        qi sym
        dqi sym
        ddqi sym
        
    end
    methods 
        function this = Links()
            %boh
        end
    

        function [] = set_inertia_matrix(this)
            m = this.mass;
            J = this.set_jacobian(); % slide 17 s3_dynamics
            Jp = J(1:3,:);
            Jo = J(4:6,:);
            %T = this.robot.T_base{(this.index + 1)}; % the attached frame corresponds to the one after the link (ex: link 3 -> ee frame)
            
            this.set_inertia_com;
            %R = T(1:3,1:3);
            this.B = m * (Jp' * Jp) + (Jo' * this.I_i * Jo);
        end

        function [] = set_robot(this,robot)
          this.robot = robot;
        end

        function [] = set_position(this,index,robot,q,dq,ddq)
            T = robot.T_base{index+1};
            R = T(1:3,1:3);
            %position com wrt 
            this.com_base = R * this.t_vec + T(1:3,4);
            T2 = robot.T_base{index};
            %position of previous joint wrt base frame
            this.prev_joint_basef = T2(1:3,4);
            this.qi = q;
            this.dqi = dq;
            this.ddqi = ddq;
        end


        function [] = set_inertia_tensor(this) % Ic in the slides
            m = this.mass;
            % check report to see in matrix form, get the inertia I
            if this.name == "rec"
                a = this.a;
                b = this.b;
                c = this.c;
                I = [
                (1/12) * m * (b^2 + c ^2)
                (1/12) * m * (a^2 + c ^2)
                (1/12) * m * (a^2 + b ^2)
                ];
            else
                a = this.radius;
                b = 0;
                h = this.length;

                I = [
                (1/2) * m * (a^2 + b^2)
                (1/2) * m * (3 * ((a^2 + b^2)^2) + h^2)
                (1/2) * m * (3 * ((a^2 + b^2)^2) + h^2)
                ];

            end

            Ic = diag(I);
            I_s = this.set_steiner; % get steiner contribution
            this.I_aug = Ic;
            this.I_i = Ic + I_s; % get total inertia

        end
        function I = set_steiner(this)

            
            r = this.t_vec; % Center of mass pli with respect to pli frame
            I = this.mass * (r' * r * eye(3) - r * r'); %steiner formula
        end

        % Get potential energy for the link
        function u = link_pot_en(this)

            syms g real
            g0 = [0;0;-9.81];
            u = this.mass * g0'*this.t_vec_sigma_zero;
            this.pot_energy_link = vpa(simplify(u),2);

        end

    end

end

        