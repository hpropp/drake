classdef BipedalSLIPDoubleSupport < DrakeSystem
    
    % where both legs are on the ground
    
    properties
        
        l0_1 = 1; % rest length of leg spring #2 (m)
        l0_2 = 1; % rest length of leg spring #2 (m)
        l_max = 1.5; % the maximum leg length (m)
        
        m1 = 1; % mass (kg)
        k = 100; % stiffness spring coefficient (aka elastic restoring force)
        g = 9.81; % gravity (m/s^2)
        alpha0; % the fixed leg orientation (for swing phases) with respect to gravity, the range of the angle of attack
        % alpha0 is the angle of attack
        
        x_last;
        E_s; % the constant system energy of the conservative spring-mass model
        
    end
    
    methods
        function obj = BipedalSLIPDoubleSupport(obj)
            %typecheck(slip, 'BipedalSLIP');
            
            obj = obj@DrakeSystem(...
                4,... % number of continuous states
                0, ... % number of discrete states
                1, ... % number of inputs
                4, ... % number of outputs
                true, ... % direct feedthrough
                true); % time invariant
            
%             obj.l0_1 = slip.l0_1;
%             obj.l0_2 = slip.l0_2;
%             obj.l_max = slip.l_max;
%             
%             obj.m1 = slip.m1;
%             obj.k = slip.k;
%             obj.g = slip.g;
%             obj.alpha0 = slip.alpha0;
%             
%             obj.x_last = slip.x_last;
%             obj.E_s = slip.E_s;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPDoubleSupport',4,'x',{'x','y','xdot','ydot'}));
            
            %obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getStateFrame(obj));
        end
        
        function  x0=getInitialState(~)
            x0=[0.5;1;0;0];
        end
        
         function y = output(obj,t,x,u)
            y = x;
         end
        
        function xdot = dynamics(obj,~,x,~) %(obj,t,x,u)
            d1 = 0.25;
            d2 = 1;
            r1 = sqrt((x(1)-d1)^2+x(2)^2);
            r2 = sqrt((x(1)-d2)^2+x(2)^2);
            theta1 = atan2(x(2),d1-x(1));
            theta2 = atan2(x(2),d2-x(1));
            
            F1 = [obj.k*(r1-obj.l0_1)*cos(theta1);-obj.k*(r1-obj.l0_1)*sin(theta1)];
            F2 = [obj.k*(r2-obj.l0_2)*cos(theta2);-obj.k*(r2-obj.l0_2)*sin(theta2)];
            F3 = [0;-obj.m1*obj.g];
            
            xdot = [x(3:4);(F1+F2+F3)/obj.m1];
            
            %             the three independent parameters:
            %             d_k = (obj.k*obj.l0_1)/(obj.m1*obj.g); % the dimentionless spring stiffness
            %             dE_s = obj.E_s/(obj.m1*obj.g*obj.l0_1); % the dimentionless system energy
            %
            %             if y == (obj.l0_1*sin(obj.alpha0))
            %                 obj.x_last = x-(obj.l0_1*cos(obj.alpha0)); % the toe x position is x+l0cos(alpha0)
            %                 d = x-obj.x_last; % x2-x1 the foot point position in the x coordinate plane
            %                 the coordinates of the hip at all times are: (x+l0cos(alpha0),y-l0sin(alpha0))
            %             end
            %
            %             P = obj.k*(obj.l0_1/sqrt(x^2+y^2)-1);
            %             Q = obj.k*(obj.l0_1/sqrt((d-x)^2+y^2)-1);
            %
            %             M_x = obj.m1*xdoubledot & (P*x)-(Q*(d-x));
            %             M_y = obj.m1*ydoubledot & (P*y)+(Q*y)-(obj.m1*obj.g);
        end
        
        %         function [T,U] = energy(obj,x)
        %
        %         end
        
%         function thetadot = orbit(obj,theta,ePE)
%             if nargin<3
%                 ePE = obj.k*(obj.l_max-obj.l0_1); % hooke's law of the elastic potential energy, (l-l0) is the displacement of the leg length
%             end % homoclinic
%             thetadot = sqrt(2*(ePE - obj.m1*obj.g*obj.l0*cos(theta))./(obj.m1*obj.l0^2));
%         end
        
%         function x0 = getInitialState(~) %(obj)
%             x0 = [0.1*randn; 20*randn; 0];
%         end
    end
end