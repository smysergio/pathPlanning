
classdef obstacle
    properties
        loc
        pobj
        po
        circle
        Kr1
        Kr2
    end
    methods
        function obj = obstacle(loc, pobj)
            global Kr1 Kr2;
            obj.loc = loc;
            obj.pobj = pobj;
            obj.po = pobj+0.025;
            obj.Kr1 = Kr1*(1-loc(1));
            obj.Kr2 = Kr2;
            th = 0:pi/50:2*pi;
            obj.circle(1,:,1) = obj.po * cos(th) + loc(1);
            obj.circle(1,:,2) = obj.po * sin(th) + loc(2);
            obj.circle(2,:,1) = pobj * cos(th) + loc(1);
            obj.circle(2,:,2) = pobj * sin(th) + loc(2);
        end
        function dUr = Fr(obj, x)
            
            p = sqrt(abs(x(1) - obj.loc(1))^2 + abs(x(2) - obj.loc(2))^2);
            if ( p < obj.po )
                if ( p < obj.pobj )
                    K = obj.Kr1;
                else
                    K = obj.Kr2;
                end
                pr = ((x(1) - obj.loc(1))^2 + (x(2) - obj.loc(2))^2)^(1/2);
                dUr(1) = (K*(2*x(1) - 2*obj.loc(1))*(1/obj.po - 1/(pr)) / (2*pr^3));
                dUr(2) = (K*(2*x(2) - 2*obj.loc(2))*(1/obj.po - 1/(pr)) / (2*pr^3));
                
                if( (dUr(1)-dUr(2)) < 0.2 )
                    dUr(2) = dUr(2)*1.1;
                end
            else
                dUr(1) = 0;
                dUr(2) = 0;   
            end
        end
    end
end
