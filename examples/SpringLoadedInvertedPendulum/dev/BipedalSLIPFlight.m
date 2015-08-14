classdef BipedalSLIPFlight < DrakeSystem
    % where both legs are in the air
    % if no flight function used, it is walking; if there is one, then running
    
    properties
        rest_l1; % rest length of leg spring #2 (m)
        rest_l2; % rest length of leg spring #2 (m)
        m_hip; % mass (kg)
        g; % gravity (m/s^2)
        
        xfoot1; % x position of r1
        xfoot2; % x position of r2
        yfoot1; % y position of r1
        yfoot2; % y position of r2
    end
    
    methods
        function obj=BipedalSLIPFlight(slip)
            typecheck(slip,'BipedalSLIP');
            
            obj=obj@DrakeSystem(...
                4, ... % number of continuous states
                0, ... % number of discrete states
                1, ... % number of inputs
                12, ... % number of outputs
                true, ... % direct feedthrough
                true); % time invariant
            
            obj.rest_l1 = slip.rest_l1;
            obj.rest_l2 = slip.rest_l2;
            obj.m_hip = slip.m_hip;
            obj.g = slip.g;
            
            obj.xfoot1 = slip.xfoot1; % x position of r1
            obj.xfoot2 = slip.xfoot2; % x position of r2
            obj.yfoot1 = slip.yfoot1; % y position of r1
            obj.yfoot2 = slip.yfoot2; % y position of r2
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPFlight',4,'x',{'x','y','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function xdot = dynamics(obj,~,x,~) %(obj,t,x,u)
            F3 = [0;-obj.m_hip*obj.g];
            xdot = [x(3:4);(F3)/obj.m_hip];
        end
        
        function y = output(obj,~,x,~) %(obj,t,x,u)
            r1 = obj.rest_l1;
            r2 = obj.rest_l2;
            theta1 = atan2(sqrt((r1^2)-((x(1)-obj.xfoot1)^2)),obj.xfoot1-x(1));
            theta2 = atan2(sqrt((r2^2)-((x(1)-obj.xfoot2)^2)),obj.xfoot2-x(1));
            
            r1dot = 0;
            theta1dot = x(3)./sqrt((r1^2)-((x(1)-obj.xfoot1)^2));
            r2dot = 0;
            theta2dot = x(3)./sqrt((r2^2)-((x(1)-obj.xfoot2)^2));
            
            y = [x(1:2);r1;theta1;r2;theta2;x(3:4);r1dot;theta1dot;r2dot;theta2dot];
        end
        
        function x0 = getInitialState(~) %(obj)
            x0 = [0.1*randn;20*randn;0];
        end
    end
end