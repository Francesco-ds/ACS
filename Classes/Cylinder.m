classdef Cylinder < Links
    properties 
        name string
        radius double
        length double

    end

    methods 
        function this = Cylinder(name,radius,length,com,index)
            this.name = name;
            this.radius = radius;
            this.length = length;
            this.t_vec = com';
            this.index = index;
        end


    end

    methods
        
        %sets the mass of its link via volume
        function[] = set_mass(this)
            volume =  (this.radius^2) * pi * this.length;
            this.mass = volume * this.density;

        end

    end








end