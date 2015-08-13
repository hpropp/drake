classdef BipedalSLIPSingleSupport < DrakeSystem
    % where one leg is on the ground
    
    properties
        rest_l1; % rest length of leg spring #2 (m)
        rest_l2; % rest length of leg spring #2 (m)
        m_hip; % mass (kg)
        k; % stiffness spring coefficient (aka elastic restoring force)
        g; % gravity (m/s^2)
        alpha0; % the fixed leg orientation (for swing phases) with respect to gravity aka the angle of attack
    
        xfoot1; % x position of r1
        xfoot2; % x position of r2
        yfoot1; % y position of r1
        yfoot2; % y position of r2
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
            
            obj.rest_l1 = slip.rest_l1;
            obj.rest_l2 = slip.rest_l2;
            obj.m_hip = slip.m_hip;
            obj.k = slip.k;
            obj.g = slip.g;
            obj.alpha0 = slip.alpha0;
            
            obj.xfoot1 = slip.xfoot1; % x position of r1
            obj.xfoot2 = slip.xfoot2; % x position of r2
            obj.yfoot1 = slip.yfoot1; % y position of r1
            obj.yfoot2 = slip.yfoot2; % y position of r2
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPSingleSupport',4,'x',{'x','y','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function  x0=getInitialState(obj)
            x0=[0.45;1;0;0];
            obj.yfoot1 = 0;
            obj.xfoot1 = 0.25;
            
            dynamics(obj,t,x,u);
            function xdot = dynamics(obj,~,x,~) %(obj,t,x,u)
                if obj.yfoot1 == 0 || obj.yfoot2~=0 
                    r1 = sqrt((x(1)-obj.xfoot1)^2+x(2)^2);
                    r2 = obj.rest_l2;
                    
                    theta1 = atan2(x(2),obj.xfoot1-x(1)); 
                    theta2 = obj.alpha0;
                    
                    % obj.xfoot1 = x(1)+((x(2)*cos(theta1))/sin(theta1));
                    obj.xfoot2 = x(1)+(r2*cos(theta2));
                    obj.yfoot2 = x(2)-(r2*sin(theta2));
                    
                    F1 = [obj.k*(r1-obj.rest_l1)*cos(theta1);-obj.k*(r1-obj.rest_l1)*sin(theta1)];
                    F3 = [0;-obj.m_hip*obj.g];
                    xdot = [x(3:4);(F1+F3)/obj.m_hip];
                    
                elseif obj.yfoot2 == 0 || obj.yfoot1~=0
                    r1 = obj.rest_l1;
                    r2 = sqrt((x(1)-obj.xfoot2)^2+x(2)^2);
                    
                    theta1 = obj.alpha0;
                    theta2 = atan2(x(2),obj.xfoot2-x(1)); 
                    
                    obj.xfoot1 = x(1)+(r1*cos(theta1));
                    obj.yfoot1 = x(2)-(r1*sin(theta1));
                    % obj.xfoot2 = x(1)+((x(2)*cos(theta2))/sin(theta2));
                    
                    F2 = [obj.k*(r2-obj.rest_l2)*cos(theta2);-obj.k*(r2-obj.rest_l2)*sin(theta2)];
                    F3 = [0;-obj.m_hip*obj.g];
                    xdot = [x(3:4);(F2+F3)/obj.m_hip];
                end
                
                output(obj,t,x,u);
                function y = output(~,~,x,~) %(obj,t,x,u)
                    if obj.yfoot1 == 0
                        r1dot = (((x(1)-obj.xfoot1)*x(3))+(x(2)*x(4)))/sqrt((x(1)^2)-(2*obj.xfoot1*x(1))+(x(2)^2)+(obj.xfoot1^2));
                        theta1dot = (((obj.xfoot1-x(1))*x(4))+(x(2)*x(3)))/((obj.xfoot1^2)-(2*obj.xfoot1*x(1))+(x(1)^2)+(x(2)^2));
                        r2dot = 0;
                        theta2dot = 0;
                    elseif obj.yfoot2 == 0
                        r1dot = 0;
                        theta1dot = 0;
                        r2dot = (((x(1)-obj.xfoot2)*x(3))+(x(2)*x(4)))/sqrt((x(1)^2)-(2*obj.xfoot2*x(1))+(x(2)^2)+(obj.xfoot2^2));
                        theta2dot = (((obj.xfoot2-x(1))*x(4))+(x(2)*x(3)))/((obj.xfoot2^2)-(2*obj.xfoot2*x(1))+(x(1)^2)+(x(2)^2));
                    end
                    
                    y = [x(1:2);r1;theta1;r2;theta2;x(3:4);r1dot;theta1dot;r2dot;theta2dot];
                end
            end
        end
    end
end