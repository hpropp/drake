classdef BipedalSLIPSingleSupport < DrakeSystem
    % where one leg is on the ground
    
    properties
        r_rest; % rest length of leg springs (m)
        m_hip; % mass (kg)
        k; % stiffness spring coefficient (aka elastic restoring force)
        g; % gravity (m/s^2)
        
        r1atground;
        r2atground;
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
            
            obj.r1atground = slip.r1atground;
            obj.r2atground = slip.r2atground;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPSingleSupport',6,'x',{'x','y','xfoot1','yfoot1','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function  x0=getInitialState(~) %(obj)
            x0=[0.45;1;0.35;0;3;3];
        end
        
        function xdot = dynamics(obj,~,x,~) %(obj,t,x,u)
            if obj.r1atground == true
                r1 = sqrt((x(1)-x(3))^2+x(2)^2);
                theta1 = atan2(x(2),x(3)-x(1));
                
                F1 = [obj.k*(r1-obj.r_rest)*cos(theta1);-obj.k*(r1-obj.r_rest)*sin(theta1)];
                F3 = [0;-obj.m_hip*obj.g];
                xdot = [x(5:6);0;0;(F1+F3)/obj.m_hip];
            elseif obj.r2atground == true
                r2 = sqrt((x(1)-x(3))^2+x(2)^2);
                theta2 = atan2(x(2),x(3)-x(1));
                
                F2 = [obj.k*(r2-obj.r_rest)*cos(theta2);-obj.k*(r2-obj.r_rest)*sin(theta2)];
                F3 = [0;-obj.m_hip*obj.g];
                xdot = [x(5:6);0;0;(F2+F3)/obj.m_hip];
            end
        end
        
        function y = output(obj,~,x,u) %(obj,t,x,u)
            if obj.r1atground == true
                r1 = sqrt((x(1)-x(3))^2+x(2)^2);
                r2 = obj.r_rest;
                theta1 = atan2(x(2),x(3)-x(1));
                theta2 = u;
                
                r1dot = (((x(1)-x(3))*x(5))+(x(2)*x(6)))/sqrt((x(1)^2)-(2*x(3)*x(1))+(x(2)^2)+(x(3)^2));
                theta1dot = (((x(3)-x(1))*x(6))+(x(2)*x(5)))/((x(3)^2)-(2*x(3)*x(1))+(x(1)^2)+(x(2)^2));
                r2dot = 0;
                theta2dot = 0;
                
                y = [x(1:2);r1;theta1;r2;theta2;x(5:6);r1dot;theta1dot;r2dot;theta2dot];
            elseif obj.r2atground == true
                r1 = obj.r_rest;
                r2 = sqrt((x(1)-x(3))^2+x(2)^2);
                theta1 = u;
                theta2 = atan2(x(2),x(3)-x(1));
                
                r1dot = 0;
                theta1dot = 0;
                r2dot = (((x(1)-x(3))*x(5))+(x(2)*x(6)))/sqrt((x(1)^2)-(2*x(3)*x(1))+(x(2)^2)+(x(3)^2));
                theta2dot = (((x(3)-x(1))*x(6))+(x(2)*x(5)))/((x(3)^2)-(2*x(3)*x(1))+(x(1)^2)+(x(2)^2));
                
                y = [x(1:2);r1;theta1;r2;theta2;x(5:6);r1dot;theta1dot;r2dot;theta2dot];
            end
        end
    end
end