classdef BipedalSLIPSingleSupport < DrakeSystem % where one leg is on the ground
    
    properties
        m = 80; % mass (kg)
        r0; % rest length of leg spring (m)
        k = 14000; % stiffness spring coefficient
        g = 9.81; % gravity (m/s^2)
        a0; % the constant orientation (for swing phases) with respect to gravity
        l = 1;
    end
    
    methods
        function obj = BipedalSLIPSingleSupport(slip)
            typecheck(slip, 'BipedalSLIP');
            
            obj = obj@DrakeSystem(...
                4, ... % number of continuous states
                0, ... % number of discrete states
                1, ... % number of inputs
                12, ... % number of outputs
                true, ... % direct feedthrough
                true); % time invariant 
            obj.m = slip.m;
            obj.r0 = slip.r0;
            obj.k = slip.k;
            obj.g = slip.g;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPSingleSupport',4,'x',{'x','y','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        % if left single support
        % then ____
        % else if right single support
        % then ____
        
        
        function [xdot,df] = dynamics(obj,~,x,~) %(obj,t,x,u)
            xdot = [x(2,:); obj.g*sin(x(1,:))/obj.l; 0*x(3,:)];
            if nargout>1,
                df = [zeros(3,1), [0, 1, 0; obj.g*cos(x(1))/obj.l, 0, 0; zeros(1,3)]];
            end
        end
        
        function thetadot = orbit(obj,theta,E)
            if nargin<3
                E = obj.m*obj.g*obj.l;
            end % homoclinic
            thetadot = sqrt(2*(E - obj.m*obj.g*obj.l*cos(theta))./(obj.m*obj.l^2));
        end
        
        function [y,dy] = output(~,~,x,~) %(obj,t,x,u)
            y = x;
            dy = [zeros(3,1),eye(3)];
        end
        
        function x0 = getInitialState(~) %(obj)
            x0 = [0.1*randn; 20*randn; 0];
        end
        % need to keep looking through the papers to find the right
        % equation that goes with this function, find
        % out the right inputs as well...
        % THE FUNCTION IS FOR WHEN THE MODEL IS ONLY ON ONE LEG, THEN THE
        % DOUBLE SUPPORT WILL BE FOR WHEN IT'S TWO!!!!! IN GEYER PDF, Look at Appendix A-D
        % and Figure 2 and Figure 4 description, 6: Discussion Walking vs
        % running = single and double support vs flight and stance running
        % THEN, IN NATURE PDF, look at methods and flight vs stance:
        % keywords = single, double, flight, stance
    end
end