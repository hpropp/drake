classdef BipedalSLIPFlight < DrakeSystem
    % where both legs are in the air
    % if no flight function used, it is walking; if there is one, then running
    
    properties
        m_hip = 1; % mass (kg)
        g = 9.81; % gravity (m/s^2)
    end
    
    methods
        function obj=BipedalSLIPFlight(slip)
            typecheck(slip,'BipedalSLIP');
            
            obj=obj@DrakeSystem(...
                4, ... % number of continuous states
                0, ... % number of discrete states
                1, ... % number of inputs
                4, ... % number of outputs
                true, ... % direct feedthrough
                true); % time invariant
            
            obj.m_hip = slip.m_hip;
            obj.g = slip.g;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPFlight',4,'x',{'x','y','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function xdot = dynamics(obj,~,x,~) %(obj,t,x,u), u because need to control the leg angles
            r1 = 1;
            r2 = 1;
            
            xfoot1 = 0.75; % x position of r1
            xfoot2 = 1.25; % x position of r2
            yfoot1 = x(2)-sqrt((r1^2)-((x(1)-xfoot1)^2)); % y position of r1
            yfoot2 = x(2)-sqrt((r1^2)-((x(1)-xfoot2)^2)); % y position of r2
            
            theta1 = atan2(sqrt((r1^2)-((x(1)-xfoot1)^2)),xfoot1-x(1));
            theta2 = atan2(sqrt((r2^2)-((x(1)-xfoot2)^2)),xfoot2-x(1));
            
            F3 = [0;-obj.m_hip*obj.g];
            xdot = [x(3:4);(F3)/obj.m_hip];
        end
        
        function y = output(~,~,x,~) %(obj,t,x,u)
            y = x;
        end
        
        function x0 = getInitialState(~) %(obj)
            x0 = [0.1*randn; 20*randn; 0];
        end
    end
end