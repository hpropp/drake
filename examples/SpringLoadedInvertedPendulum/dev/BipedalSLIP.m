classdef BipedalSLIP < HybridDrakeSystem
    % a script combining all of the other classes
    % spring loaded inverted pendulum, the control input (aka 'u') is the 'angle of
    % attack'
    
    % use if y==l0 for taking off and switching between single and double
    % and write the guards first, then the transitions, look at
    % hybriddrakesystem, line 101
    
    properties
        rest_l1 = 1; % rest length of leg spring #2 (m)
        rest_l2 = 1; % rest length of leg spring #2 (m)
        m_hip = 1; % mass (kg)
        k = 100; % stiffness spring coefficient (aka elastic restoring force)
        g = 9.81; % gravity (m/s^2)
        alpha0 = pi/3; % the fixed leg orientation (for swing phases) with respect to gravity aka the angle of attack
        
        stopAtApex = 0;
    end
    
    methods
        function obj = BipedalSLIP()
            obj = obj@HybridDrakeSystem(...
                1, ... % number of inputs
                12); % number of outputs
            
            obj = setInputFrame(obj,CoordinateFrame('SLIPInput',1,'u',{'alpha0'})); % alpha0 is the angle of attack
            obj = setOutputFrame(obj,CoordinateFrame('SLIPOutput',12,'y',{'x','y','r1','theta1','r2','theta2','xdot','ydot','r1dot','theta1dot','r2dot','theta2dot'}));
            
            % flight mode, where both legs are in the air
            pFlight=BipedalSLIPFlight(obj);
            [obj, pf_mode]=obj.addMode(pFlight);
            
            % singleSupport mode, where one leg is on the ground
            pSingleSupport=BipedalSLIPSingleSupport(obj);
            [obj, ps_mode]=obj.addMode(pSingleSupport);
            
            % doubleSupport mode, where both legs are on the ground
            pDoubleSupport=BipedalSLIPDoubleSupport(obj);
            [obj, pd_mode]=obj.addMode(pDoubleSupport);
            
            if obj.alpha0 >= pi/3 % the model is walking
                obj=addTransition(obj,ps_mode,@single2doubleGuard,@single2double,true,true,pd_mode);
                obj=addTransition(obj,pd_mode,@double2SingleGuard,@double2single,true,true,ps_mode);
            elseif obj.alpha0 < pi/3 % the model is running
                obj=addTransition(obj,ps_mode,@stance2flightGuard,@stance2flight,true,true,pf_mode);
                obj=addTransition(obj,pf_mode,@flight2stanceGuard,@flight2stance,true,true,ps_mode);
            end
        end
        
        function [g,dg]=single2doubleGuard(obj,~,x,~) %(obj,t,x,u)
            g=x(2)-obj.rest_l1*cos(obj.alpha0);  % foot hits the ground
            dg=[0 1 0 0 0 0 zeros(1,6)];
        end
        
        function [g,dg]=double2SingleGuard(obj,~,x,~) %(obj,t,x,u)
            g=obj.rest_l1-x(3);  % r >= l0, x3 = r1
            dg=[0 -1 0 0 0 0 zeros(1,6)];
        end
        
        function [g,dg]=stance2flightGuard(~,~,x,~) %(obj,t,x,u)
            g=-x(9);  % rdot >= 0 x9 = r1dot
            dg=[zeros(1,6) 0 0 -1 0 0 0];
        end
        
        function [g,dg]=flight2stanceGuard(~,~,x,~) %(obj,t,x,u)
            g=x(8);  % ydot <= 0
            dg=[0 0 0 0 0 0 0 0 1 0 0 0 0];
        end
        
        function [xp,mode,status,dxp]=stance2flight(~,mode,~,xm,~) %(obj,t,x,u)
            xp=[xm(5)-xm(1)*sin(xm(2));...
                xm(1)*cos(xm(2));...
                -xm(3)*sin(xm(2))-xm(1)*xm(4)*cos(xm(2));...
                xm(3)*cos(xm(2))-xm(1)*xm(4)*sin(xm(2));];
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=3;
            end
            status=(xp(3)<0);  % terminate if xdot < 0
            dxpdxm=zeros(4,5);
            dxpdxm(1,5)=1;
            dxpdxm(1,1)=-sin(xm(2));
            dxpdxm(1,2)=-xm(1)*cos(xm(2));
            dxpdxm(1,5)=1;
            dxpdxm(2,1)=cos(xm(2));
            dxpdxm(2,2)=-xm(1)*sin(xm(2));
            dxpdxm(2,6)=1;
            dxpdxm(3,1)=-xm(4)*cos(xm(2));
            dxpdxm(3,2)=-xm(3)*cos(xm(2))+xm(1)*xm(4)*sin(xm(2));
            dxpdxm(3,3)=-sin(xm(2));
            dxpdxm(3,4)=-xm(1)*cos(xm(2));
            dxpdxm(4,1)=-xm(4)*sin(xm(2));
            dxpdxm(4,2)=-xm(3)*sin(xm(2))-xm(1)*xm(4)*cos(xm(2));
            dxpdxm(4,3)=cos(xm(2));
            dxpdxm(4,4)=-xm(1)*sin(xm(2));
            dxp=[zeros(4,2) dxpdxm zeros(4,1)];
        end
        
        function [xp,mode,status,dxp]=flight2stance(~,mode,~,xm,u) %(obj,mode,t,xm,u)
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=2;
            end
            theta=u;
            r= xm(2)/cos(theta); % = obj.rest_l1 because x(2)-obj.rest_l1*cos(u) = 0
            xp=[r;...
                theta;...
                -xm(3)*sin(theta)+xm(4)*cos(theta);...
                -(xm(3)*cos(theta)+xm(4)*sin(theta))/r;...
                xm(1)+r*sin(theta);0;0;0;0;0;0;0]; %xm(2)*tan(theta)];
            status=0;
            if nargout>3
                dxpdxm=zeros(5,4);
                dxpdu=zeros(5,1);
                dxpdxm(1,2)=1/cos(theta);
                dxpdu(1,1)=(xm(2))*sin(theta)/cos(theta)^2;
                %dxpdxm(1,6)=-1/cos(theta);
                dxpdu(2,1)=1;
                dxpdxm(3,3)=-sin(theta);
                dxpdxm(3,4)=cos(theta);
                dxpdu(3,1)=-xm(3)*cos(theta)-xm(4)*sin(theta);
                dxpdxm(4,2)=(xm(3)*cos(theta)^2+xm(4)*sin(theta)*cos(theta))/(xm(2))^2;
                dxpdxm(4,3)=-cos(theta)^2/(xm(2));
                dxpdxm(4,4)=-sin(theta)*cos(theta)/(xm(2));
                dxpdu(4,1)=-(-xm(3)*sin(2*theta)+xm(4)*cos(2*theta))/(xm(2));
                %dxpdxm(4,6)=-dxpdxm(4,2);
                dxpdxm(5,1)=1;
                dxpdxm(5,2)=tan(theta);
                %error('still need to update this last one:');
                dxpdu(5,1)=xm(2)/cos(theta)^2;
                dxp=[zeros(5,2) dxpdxm dxpdu];
            end
        end
        
        function theta=angleOfAttackControl(obj,yd,y0,xdot0)
            aoaControl=@(theta) obj.apex2apex(theta,y0,xdot0)-yd;
            theta=fzero(aoaControl,pi/8);
        end
    end
    
    methods (Static)
        function run(alpha0)
            r = BipedalSLIP();
            vis = BipedalSLIPVisualizer(r);
            
            aoa_command = setOutputFrame(ConstantTrajectory(alpha0),getInputFrame(r));
            ytraj = simulate(cascade(aoa_command,r),[0 11]);
            vis.playback(ytraj,struct('slider','true'));
        end
    end
end