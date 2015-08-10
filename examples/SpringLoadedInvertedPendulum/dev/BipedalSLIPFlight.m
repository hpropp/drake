classdef BipedalSLIPFlight < DrakeSystem
    
    % where both legs are in the air
    % if no flight function used, it is walking; if there is one, then running
    
    properties
        l0; % allowing to sitch from l0_1 to l0_2 in single support left vs. right controlling
        l0_1 = 1; % rest length of leg spring #2 (m)
        l0_2 = 1; % rest length of leg spring #2 (m)
        l_max = 1.5; % the maximum leg length (m)
        
        m1 = 80; % mass (kg)
        k = 14000; % stiffness spring coefficient (aka elastic restoring force)
        g = 9.81; % gravity (m/s^2)
        alpha0; % the fixed leg orientation (for swing phases) with respect to gravity, the range of the angle of attack
        % alpha0 is the angle of attack
        
        x_last;
        E_s; % the constant system energy of the conservative spring-mass model
    end
    methods
        
        function obj=BipedalSLIPFlight(slip)
            typecheck(slip,'BipedalSLIP');
            
            obj=obj@DrakeSystem(...
                12, ... % number of continuous states
                0, ... % number of discrete states
                1, ... % number of inputs
                12, ... % number of outputs
                true, ... % direct feedthrough
                true); % time invariant
            
            obj.l0 = slip.l0;
            obj.l0_1 = slip.l0_1;
            obj.l0_2 = slip.l0_2;
            obj.l_max = slip.l_max;
            
            obj.m1 = slip.m1;
            obj.k = slip.k;
            obj.g = slip.g;
            obj.alpha0 = slip.alpha0;
            
            obj.x_last = slip.x_last;
            obj.E_s = slip.E_s;
            
            obj = setStateFrame(obj,CoordinateFrame('BipedalSLIPFlight',12,'x',{'x','y','r1','theta1','r2','theta2','xdot','ydot','r1dot','theta1dot','r2dot','theta2dot'}));
            
            obj = setInputFrame(obj,getInputFrame(slip));
            obj = setOutputFrame(obj,getOutputFrame(slip));
        end
        
%         function  x0=getInitialState(~)
%             x0=[0;1;1;0;1;0;0;0;0;0;0;0];
%         end
        
        %         function obj=setInitialState(obj,y0)
        %             obj.x0=[0;y0;sqrt(2*(obj.E-obj.m1*obj.g*y0)/obj.m1);0];
        %         end
        
                
%         function [T,U] = energy(obj,x)
%             L = (.5*obj.m1);*(xdot.^2))-(obj.m1*obj.g); % NOTE: THIS IS NOT CORRECT CURRENTLY
%         end
        
        
        function [xdot,df]=dynamics(obj,~,x,u) %(obj,t,x,u), u because need to control the angles of the legs
            xdot=[x(7:12);0;-obj.g;0;0;0;0];
            
            if(nargout>1)
                dfdx=[zeros(2) eye(2);zeros(2,4)];
                df=[zeros(4,1) dfdx zeros(4,1)];
            end
        end
        
        function [y1,y2]=output(obj,~,x,u) %(obj,t,x,u)
            y1=[x(1:2);obj.l0_1;u;x(3:4);0;0];
            y2=[x(1:2);obj.l0_2;u;x(3:4);0;0];
        end
    end
end