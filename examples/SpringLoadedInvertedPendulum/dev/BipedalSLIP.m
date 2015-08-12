classdef BipedalSLIP < HybridDrakeSystem
    % a script combining all of the other classes
    % spring loaded inverted pendulum, the control input (aka 'u') is the 'angle of
    % attack'
    
    properties
        rest_l1 = 1; % rest length of leg spring #2 (m)
        rest_l2 = 1; % rest length of leg spring #2 (m)
        
        m_hip = 1; % mass (kg)
        k = 100; % stiffness spring coefficient (aka elastic restoring force)
        g = 9.81; % gravity (m/s^2)
        alpha0 = pi/3; % the fixed leg orientation (for swing phases) with respect to gravity aka the angle of attack
        
        stopAtApex = 0;
    end
    
    properties (Constant)
        % these values go with the static methods and will say whether we are walking or running
        walking_v = 1.5; % an example of a walking velocity (m/s)
        walking_angle = 76; % an example of a walking angle of attack (degrees)
        running_v = 3; % an example of a running velocity (m/s)
        running_angle = 81; % an example of a running angle of attack (degrees) NOTE: THE CODE DEOS NOT LIKE 81, CHANGE TO DIF #
    end

    methods
        function obj = BipedalSLIP()
            
            obj = obj@HybridDrakeSystem(...
                1, ... % number of inputs
                12); % number of outputs
            
            obj = setInputFrame(obj,CoordinateFrame('SLIPInput',1,'u',{'alpha0'})); % alpha0 is the angle of attack
            obj = setOutputFrame(obj,CoordinateFrame('SLIPOutput',12,'y',{'x','y','r1','theta1','r2','theta2','xdot','ydot','r1dot','theta1dot','r2dot','theta2dot'}));
            
            pFlight=BipedalSLIPFlight(obj);
            [obj, pf_mode]=obj.addMode(pFlight); % where both legs are in the air
            
            pSingleSupport=BipedalSLIPSingleSupport(obj); % where one leg is on the ground
            [obj, ps_mode]=obj.addMode(pSingleSupport);
            
            pDoubleSupport=BipedalSLIPDoubleSupport(obj); % where both legs are on the ground
            [obj, pd_mode]=obj.addMode(pDoubleSupport);
            
            obj=addTransition(obj,pf_mode,@collisionGuard,@flight2stance,true,true);
            obj=addTransition(obj,ps_mode,andGuards(obj,@takeOffGuard1A,@takeOffGuard1B),@stance2flight,false,true);
            % running mode
            
%             obj=addTransition(obj,pd_mode,andGuards(obj,@double2SingleGuard,@single2DoubleGuard),@double2single,true,true);
%             obj=addTransition(obj,ps_mode,@collisionGuard,@single2double,true,true);
%             % walking mode
            
            % obj = addTransition(obj,pf_mode,andGuards(obj,@flight2SingleGuard,@single2FlightGuard),@flightDynamics,true,true);
            % obj = addTransition(obj,pSingleSupport,@takeOffGuard1A,@singleSupportDynamics,true,true);
            % obj = addTransition(obj,pDoubleSupport,andGuards(obj,@double2SingleGuard,@single2DoubleGuard),@doubleSupportDynamics,true,true);
            
            % use if y==l0 for taking off and switching between single and double
        end
        
        function [g,dg]=collisionGuard(obj,~,x,u) %(obj,t,x,u)
            g=x(2)-obj.rest_l1*cos(obj.alpha0);  % foot hits the ground
            dg=[0 1 0 0 0 0 zeros(1,6)];
        end
        
        function [g,dg]=takeOffGuard1A(obj,~,x,~) %(obj,t,x,u)
            g=obj.rest_l1-x(3);  % r >= l0
            dg=[0 -1 0 0 0 0 zeros(1,6)];
        end
        
        function [g,dg]=takeOffGuard1B(~,~,x,~) %(obj,t,x,u)
            g=-x(9);  % rdot >= 0
            dg=[zeros(1,6) 0 0 -1 0 0 0];
        end
        
        function [g,dg]=apexGuard(~,~,x,~) %(obj,t,x,u)
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
        
        function [xp,mode,status,dxp]=reachApex(obj,mode,~,xm,u) %(obj,mode,t,xm,u)
            if(mode~=3)
                error('Incorrect mode');
            else
                mode=1;
            end
            xp=xm;
            if(xp(2)<obj.rest_l1*cos(u))
                error('toe touches ground at apex')
            end
            status=obj.stopAtApex;
            dxp=[zeros(4,2) eye(4),zeros(4,1)];
        end
        
        %     function xp=flight2stance(obj,x,theta)
        %         if(x(1)~=1)
        %             error('The first entry must be mode, and in stance mode');
        %         end
        %         xc=x(2:5);
        %         r=xc(2)/cos(theta);
        %         xcn=[r;theta;...
        %             -xc(3)*sin(theta)+xc(4)*cos(theta);...
        %             -(xc(3)*cos(theta)+xc(4)*sin(theta))/r];
        %         xp=[2;xcn];
        %     end
        
        function yn=apex2apex(obj,theta,y0,xdot0)
            if(nargin==2)
                x0=obj.getInitialState;
            else
                x0=[0;y0;xdot0;0];
            end
            obj.stopAtApex=1;
            yTraj=simulate(cascade(setOutputFrame(ConstantTrajectory(theta),getInputFrame(obj)),obj),[0 10],[x0;0]);
            yApex=Point(getOutputFrame(obj),yTraj.eval(yTraj.tspan(end)));
            if yApex.xdot<0
                yn = nan;
            else
                yn = yApex.y;
            end
        end
        
        function theta=angleOfAttackControl(obj,yd,y0,xdot0)
            aoaControl=@(theta) obj.apex2apex(theta,y0,xdot0)-yd;
            theta=fzero(aoaControl,pi/8);
        end
    end
    
    methods (Static)
        function run(v, alpha0) % alpha0 is the angle of attack
            
            assert(abs(v-BipedalSLIP.walking_v) < 1e-10 || abs(v-BipedalSLIP.running_v) < 1e-10);
            assert(abs(alpha0-BipedalSLIP.walking_angle) < 1e-10 || abs(alpha0-BipedalSLIP.running_angle) < 1e-10);
            
            %             if abs(velocity-obj.walking_v) < 1e-10
            %                 alpha0 = obj.walking_angle;
            %             elseif abs(velocity-obj.running_v) < 1e-10
            %                 alpha0 = obj.running_angle;
            %             end
            
            r = BipedalSLIP();
            vis = BipedalSLIPVisualizer(r);
            x0 = r.getInitialState();
            x0(7) = v;
            
            aoa_command = setOutputFrame(ConstantTrajectory(alpha0),getInputFrame(r));
            ytraj = simulate(cascade(aoa_command,r),[0 11], x0);
            vis.playback(ytraj,struct('slider','true'));
            
            
        end
    end
end