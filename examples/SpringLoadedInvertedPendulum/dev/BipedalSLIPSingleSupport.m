classdef BipedalSLIPSingleSupport < DrakeSystem
    % where one leg is on the ground
    % 'r1' is always on the ground, 'r2' is always in the air
    
    properties
        r_rest; % rest length of leg springs (m)
        m_hip; % mass (kg)
        k; % stiffness spring coefficient (aka elastic restoring force)
        g; % gravity (m/s^2)
    end
    
    methods
        function obj = BipedalSLIPSingleSupport(slip)
            typecheck(slip, 'BipedalSLIP');
            
            obj = obj@DrakeSystem(...
                6, ... % number of continuous states
                0, ... % number of discrete states
                1, ... % number of inputs
                12, ... % number of outputs
                true, ... % direct feedthrough
                true); % time invariant
            
            obj.r_rest = slip.r_rest;
            obj.m_hip = slip.m_hip;
            obj.k = slip.k;
            obj.g = slip.g;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPSingleSupport',6,'x',{'x','y','xfoot','yfoot','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function  x0=getInitialState(obj)
            x0=[0.45;1;0.3;0;3;3];
        end
        
        function xdot = dynamics(obj,t,x,u) %(obj,t,x,u)
            r1 = sqrt((x(1)-x(3))^2+x(2)^2);
            theta1 = atan2(x(2),x(3)-x(1));
            
            F1 = [obj.k*(r1-obj.r_rest)*cos(theta1);-obj.k*(r1-obj.r_rest)*sin(theta1)];
            F2 = [0;-obj.m_hip*obj.g];
            xdot = [x(5:6);0;0;(F1+F2)/obj.m_hip];
        end
        
        function y = output(obj,t,x,u) %(obj,t,x,u)
            r1 = sqrt((x(1)-x(3))^2+x(2)^2);
            r2 = obj.r_rest;
            theta1 = atan2(x(2),x(3)-x(1));
            theta2 = u;
            
            r1dot = (((x(1)-x(3))*x(5))+(x(2)*x(6)))/sqrt((x(1)^2)-(2*x(3)*x(1))+(x(2)^2)+(x(3)^2));
            theta1dot = (((x(3)-x(1))*x(6))+(x(2)*x(5)))/((x(3)^2)-(2*x(3)*x(1))+(x(1)^2)+(x(2)^2));
            r2dot = 0;
            theta2dot = 0;
            y = [x(1:2);r1;theta1;r2;theta2;x(5:6);r1dot;theta1dot;r2dot;theta2dot];
        end
    end
end