classdef BipedalSLIPSingleSupport < DrakeSystem
    % where one leg is on the ground
    
    properties
        rest_l1; % rest length of leg spring #2 (m)
        rest_l2; % rest length of leg spring #2 (m)
        m_hip; % mass (kg)
        k; % stiffness spring coefficient (aka elastic restoring force)
        g; % gravity (m/s^2)
        alpha0; % the fixed leg orientation (for swing phases) with respect to gravity aka the angle of attack
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
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPSingleSupport',4,'x',{'x','y','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function  x0=getInitialState(~) %(obj)
            x0=[0.45;1;0;0];
        end
        
        function xdot = dynamics(obj,~,x,~) %(obj,t,x,u)
            yfoot1 = 0;
            xfoot1 = 0.25;
            % yfoot2 = 0;
            % xfoot2 = 0.25;
            
            if yfoot1 == 0 % is a while(true) better?
                r1 = sqrt((x(1)-xfoot1)^2+x(2)^2);
                r2 = obj.rest_l2;
                
                theta1 = atan2(x(2),xfoot1-x(1));
                theta2 = obj.alpha0;
                
                % xfoot1 = x(1)+((x(2)*cos(theta1))/sin(theta1));
                xfoot2 = x(1)+(r2*cos(theta2));
                yfoot2 = x(2)-(r2*sin(theta2));
                
                F1 = [obj.k*(r1-obj.rest_l1)*cos(theta1);-obj.k*(r1-obj.rest_l1)*sin(theta1)];
                F3 = [0;-obj.m_hip*obj.g];
                xdot = [x(3:4);(F1+F3)/obj.m_hip];
                
            elseif yfoot2 == 0
                r1 = obj.rest_l1;
                r2 = sqrt((x(1)-xfoot2)^2+x(2)^2);
                
                theta1 = obj.alpha0;
                theta2 = atan2(x(2),xfoot2-x(1));
                
                xfoot1 = x(1)+(r1*cos(theta1));
                yfoot1 = x(2)-(r1*sin(theta1));
                % xfoot2 = x(1)+((x(2)*cos(theta2))/sin(theta2));
                
                F2 = [obj.k*(r2-obj.rest_l2)*cos(theta2);-obj.k*(r2-obj.rest_l2)*sin(theta2)];
                F3 = [0;-obj.m_hip*obj.g];
                xdot = [x(3:4);(F2+F3)/obj.m_hip];
            end
            
            output(obj,t,x,u);
            function y = output(~,~,x,~) %(obj,t,x,u)
                y = [x(1:2);r1;theta1;r2;theta2;x(3:4);0;0;0;0]; % find out values of the last four
                xfoot1;
                yfoot1;
                xfoot2;
                yfoot2;
            end
        end
    end
end