classdef BipedalSLIPDoubleSupport < DrakeSystem
    % where both legs are on the ground
    
    properties
        rest_l1; % rest length of leg spring #2 (m)
        rest_l2; % rest length of leg spring #2 (m)
        m_hip; % mass (kg)
        k; % stiffness spring coefficient (aka elastic restoring force)
        g; % gravity (m/s^2)
        
        xfoot1; % x position of r1
        xfoot2; % x position of r2
        yfoot1; % y position of r1
        yfoot2; % y position of r2
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
            
            obj.rest_l1 = slip.rest_l1;
            obj.rest_l2 = slip.rest_l2;
            obj.m_hip = slip.m_hip;
            obj.k = slip.k;
            obj.g = slip.g;
            
            obj.xfoot1 = slip.xfoot1; % x position of r1
            obj.xfoot2 = slip.xfoot2; % x position of r2
            obj.yfoot1 = slip.yfoot1; % y position of r1
            obj.yfoot2 = slip.yfoot2; % y position of r2
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPDoubleSupport',4,'x',{'x','y','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function xdot = dynamics(obj,~,x,~) %(obj,t,x,u)
            r1 = sqrt((x(1)-obj.xfoot1)^2+x(2)^2);
            r2 = sqrt((x(1)-obj.xfoot2)^2+x(2)^2);
            theta1 = atan2(x(2),obj.xfoot1-x(1));
            theta2 = atan2(x(2),obj.xfoot2-x(1));
            
            F1 = [obj.k*(r1-obj.rest_l1)*cos(theta1);-obj.k*(r1-obj.rest_l1)*sin(theta1)];
            F2 = [obj.k*(r2-obj.rest_l2)*cos(theta2);-obj.k*(r2-obj.rest_l2)*sin(theta2)];
            F3 = [0;-obj.m_hip*obj.g];
            xdot = [x(3:4);(F1+F2+F3)/obj.m_hip];
        end
        
        function y = output(obj,~,x,~) %(obj,t,x,u)
            r1 = sqrt((x(1)-obj.xfoot1)^2+x(2)^2);
            r2 = sqrt((x(1)-obj.xfoot2)^2+x(2)^2);
            theta1 = atan2(x(2),obj.xfoot1-x(1));
            theta2 = atan2(x(2),obj.xfoot2-x(1));
            
            r1dot = (((x(1)-obj.xfoot1)*x(3))+(x(2)*x(4)))/sqrt((x(1)^2)-(2*obj.xfoot1*x(1))+(x(2)^2)+(obj.xfoot1^2));
            theta1dot = (((obj.xfoot1-x(1))*x(4))+(x(2)*x(3)))/((obj.xfoot1^2)-(2*obj.xfoot1*x(1))+(x(1)^2)+(x(2)^2));
            r2dot = (((x(1)-obj.xfoot2)*x(3))+(x(2)*x(4)))/sqrt((x(1)^2)-(2*obj.xfoot2*x(1))+(x(2)^2)+(obj.xfoot2^2));
            theta2dot = (((obj.xfoot2-x(1))*x(4))+(x(2)*x(3)))/((obj.xfoot2^2)-(2*obj.xfoot2*x(1))+(x(1)^2)+(x(2)^2));
            
            y = [x(1:2);r1;theta1;r2;theta2;x(3:4);r1dot;theta1dot;r2dot;theta2dot];
        end
        
        function x0 = getInitialState(~) %(obj)
            x0 = [0.1*randn;20*randn;0];
        end
    end
end