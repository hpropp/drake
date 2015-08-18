classdef BipedalSLIPSingleSupportLeg1 < DrakeSystem
    % where one leg is on the ground
    
    properties
        r_rest; % rest length of leg springs (m)
        m_hip; % mass (kg)
        k; % stiffness spring coefficient (aka elastic restoring force)
        g; % gravity (m/s^2)
        
        xfoot1; % x position of r1
        xfoot2; % x position of r2
        yfoot1; % y position of r1
        yfoot2; % y position of r2
    end
    
    methods
        function obj = BipedalSLIPSingleSupportLeg1(slip)
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
            
            obj.xfoot1 = slip.xfoot1; % x position of r1
            obj.xfoot2 = slip.xfoot2; % x position of r2
            obj.yfoot1 = slip.yfoot1; % y position of r1
            obj.yfoot2 = slip.yfoot2; % y position of r2
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPSingleSupport',6,'x',{'x','y','xfoot1','yfoot1','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function  x0=getInitialState(~) %(obj)
            x0=[0.45;1;0.35;0;0;0];
        end
        
        function xdot = dynamics(obj,~,x,u) %(obj,t,x,u)
            obj.xfoot1 = x(3);
            obj.yfoot1 = x(4);
            
            r1 = sqrt((x(1)-x(3))^2+x(2)^2);
            r2 = obj.r_rest;
            theta1 = atan2(x(2),x(3)-x(1));
            theta2 = u;
            
            obj.xfoot2 = x(1)+(r2*cos(theta2));
            obj.yfoot2 = x(2)-(r2*sin(theta2));
            
            F1 = [obj.k*(r1-obj.r_rest)*cos(theta1);-obj.k*(r1-obj.r_rest)*sin(theta1)];
            F3 = [0;-obj.m_hip*obj.g];
            xdot = [x(5:6);0;0;(F1+F3)/obj.m_hip];
        end
        
        function y = output(obj,~,x,u) %(obj,t,x,u)
            r1 = sqrt((x(1)-x(3))^2+x(2)^2);
            r2 = obj.r_rest;
            theta1 = atan2(x(2),x(3)-x(1));
            theta2 = u;
            
            obj.xfoot1 = x(3);
            obj.yfoot1 = x(4);
            obj.xfoot2 = x(1)+(r2*cos(theta2));
            obj.yfoot2 = x(2)-(r2*sin(theta2));
            
            r1dot = (((x(1)-x(3))*x(5))+(x(2)*x(6)))/sqrt((x(1)^2)-(2*x(3)*x(1))+(x(2)^2)+(x(3)^2));
            theta1dot = (((x(3)-x(1))*x(6))+(x(2)*x(5)))/((x(3)^2)-(2*x(3)*x(1))+(x(1)^2)+(x(2)^2));
            r2dot = 0;
            theta2dot = 0;
            
            y = [x(1:2);r1;theta1;r2;theta2;x(5:6);r1dot;theta1dot;r2dot;theta2dot];
        end
    end
end