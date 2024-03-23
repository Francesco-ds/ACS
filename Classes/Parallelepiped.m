classdef Parallelepiped < Links
    properties 
        name string
        a double
        b double
        c double

    end
    methods 
        function this = Parallelepiped(name,a,b,c,com,index)
            this.name = name;
            this.a = a;
            this.b = b;
            this.c = c;
            this.t_vec = com';
            this.index = index;

        end

    end

        methods
        %sets the mass of its link via volume
        function[] = set_mass(this)
            volume =  this.a * this.b * this.c;
            this.mass = volume * this.density;

        end


        end
end
