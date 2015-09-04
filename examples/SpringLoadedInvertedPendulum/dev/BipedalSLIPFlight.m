classdef BipedalSLIPFlight < DrakeSystem
    % where both legs are in the air
    % if no flight function used, it is walking; if there is one, then running
    % 'r1' is always behind 'r2'
    
    properties
        r_rest; % rest length of leg springs (m)
        m_hip; % mass (kg)
        g; % gravity (m/s^2)
        alpha0_r2;
        alpha0_r1;
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
            
            obj.r_rest = slip.r_rest;
            obj.m_hip = slip.m_hip;
            obj.g = slip.g;
            obj.alpha0_r2 = slip.alpha0_r2;
            obj.alpha0_r1 = slip.alpha0_r1;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPFlight',4,'x',{'x','y','xdot','ydot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
        function xdot = dynamics(obj,t,x,u) %(obj,t,x,u)
            F1 = [0;-obj.m_hip*obj.g];
            xdot = [x(3:4);(F1)/obj.m_hip];
        end
        
        function y = output(obj,t,x,u) %(obj,t,x,u)
            r1 = obj.r_rest;
            r2 = obj.r_rest;
            theta1 = obj.alpha0_r1;
            theta2 = obj.alpha0_r2;
            
            r1dot = 0;
            theta1dot = 0;
            r2dot = 0;
            theta2dot = 0;
            
            y = [x(1:2);r1;theta1;r2;theta2;x(3:4);r1dot;theta1dot;r2dot;theta2dot];
        end
        
        function  x0=getInitialState(obj)
            x0 = [0.1*randn;20*randn;0];
        end
    end
end