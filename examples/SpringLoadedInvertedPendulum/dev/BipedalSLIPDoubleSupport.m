classdef BipedalSLIPDoubleSupport < DrakeSystem % where one leg is on the ground
    
    properties
        m; % mass (kg)
        r0; % rest length of leg spring (m)
        k; % stiffness spring coefficient
        g; % gravity (m/s^2)
    end
    
    methods
        function obj = BipedalSLIPDoubleSupport(slip)
            typecheck(slip, 'BipedalSLIP');
            
            obj = obj@DrakeSystem(...
                4,... % number of continuous states
                0, ... % number of discrete states
                1, ... % number of inputs
                12, ... % number of outputs
                true, ... % direct feedthrough
                true); % time invariant
            obj.m = slip.m;
            obj.r0 = slip.r0;
            obj.k = slip.k;
            obj.g = slip.g;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPDoubleSupport',4,'x',{'x','y','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
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
    end
end