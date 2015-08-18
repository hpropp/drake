classdef BipedalSLIPDoubleSupport < DrakeSystem
    % where both legs are on the ground
    
    properties
        r_rest; % rest length of leg springs (m)
        m_hip; % mass (kg)
        k; % stiffness spring coefficient (aka elastic restoring force)
        g; % gravity (m/s^2)
    end
    
    methods
        function obj = BipedalSLIPDoubleSupport(slip)
            typecheck(slip, 'BipedalSLIP');
            
            obj = obj@DrakeSystem(...
                8,... % number of continuous states
                0, ... % number of discrete states
                1, ... % number of inputs
                12, ... % number of outputs
                true, ... % direct feedthrough
                true); % time invariant
            
            obj.r_rest = slip.r_rest;
            obj.m_hip = slip.m_hip;
            obj.k = slip.k;
            obj.g = slip.g;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPDoubleSupport',8,'x',{'x','y','xfoot1','yfoot1','xfoot2','yfoot2','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function xdot = dynamics(obj,~,x,~) %(obj,t,x,u)
            r1 = sqrt((x(1)-x(3))^2+x(2)^2);
            r2 = sqrt((x(1)-x(5))^2+x(2)^2);
            theta1 = atan2(x(2),x(3)-x(1));
            theta2 = atan2(x(2),x(5)-x(1));
            
            F1 = [obj.k*(r1-obj.r_rest)*cos(theta1);-obj.k*(r1-obj.r_rest)*sin(theta1)];
            F2 = [obj.k*(r2-obj.r_rest)*cos(theta2);-obj.k*(r2-obj.r_rest)*sin(theta2)];
            F3 = [0;-obj.m_hip*obj.g];
            xdot = [x(7:8);0;0;0;0;(F1+F2+F3)/obj.m_hip];
        end
        
        function y = output(~,~,x,~) %(obj,t,x,u)
            r1 = sqrt((x(1)-x(3))^2+x(2)^2);
            r2 = sqrt((x(1)-x(5))^2+x(2)^2);
            theta1 = atan2(x(2),x(3)-x(1));
            theta2 = atan2(x(2),x(5)-x(1));
            
            r1dot = (((x(1)-x(3))*x(7))+(x(2)*x(8)))/sqrt((x(1)^2)-(2*x(3)*x(1))+(x(2)^2)+(x(3)^2));
            theta1dot = (((x(3)-x(1))*x(8))+(x(2)*x(3)))/((x(3)^2)-(2*x(3)*x(1))+(x(1)^2)+(x(2)^2));
            r2dot = (((x(1)-x(5))*x(7))+(x(2)*x(8)))/sqrt((x(1)^2)-(2*x(5)*x(1))+(x(2)^2)+(x(5)^2));
            theta2dot = (((x(5)-x(1))*x(8))+(x(2)*x(3)))/((x(5)^2)-(2*x(5)*x(1))+(x(1)^2)+(x(2)^2));
            
            y = [x(1:2);r1;theta1;r2;theta2;x(7:8);r1dot;theta1dot;r2dot;theta2dot];
        end
        
        function x0 = getInitialState(~) %(obj)
            x0 = [0.1*randn;20*randn;0];
        end
    end
end