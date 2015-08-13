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
        
        xfoot1 = []; % x position of r1
        xfoot2 = []; % x position of r2
        yfoot1 = []; % y position of r1
        yfoot2 = []; % y position of r2
    end
    
    methods
        function obj = BipedalSLIP()
            obj = obj@HybridDrakeSystem(...
                1, ... % number of inputs
                12); % number of outputs
            
            obj = setInputFrame(obj,CoordinateFrame('SLIPInput',1,'u',{'alpha0'})); % alpha0 is the angle of attack
            obj = setOutputFrame(obj,CoordinateFrame('SLIPOutput',12,'y',{'x','y','r1','theta1','r2','theta2','xdot','ydot','r1dot','theta1dot','r2dot','theta2dot'}));
            
            % flight mode (both legs in the air)
            pFlight=BipedalSLIPFlight(obj);
            [obj, pf_mode]=obj.addMode(pFlight);
            
            % singleSupport mode (one leg on the ground, one leg in the air)
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
        
        function guard_s2d = single2doubleGuard(obj,~,ps_x,~) %(obj,t,x,u)
            guard_s2d = [obj.yfoot1;obj.yfoot2];
            if theta1 == atan2(ps_x(2),obj.xfoot1-ps_x(1)) && theta2 == atan2(ps_x(2),obj.xfoot2-ps_x(1))
                obj.yfoot1 = 0;
                obj.yfoot2 = 0;
            end
        end
        
        function guard_d2s = double2SingleGuard(obj,~,pd_x,~) %(obj,t,x,u)
            if obj.xfoot1 > obj.xfoot2 && pd_x(3) == obj.rest_l1
                guard_d2s = pd_x(11);
            elseif obj.xfoot1 < obj.xfoot2 && pd_x(5) == obj.rest_l2
                guard_d2s = pd_x(9);
            end
        end
        
        function guard_s2f = stance2flightGuard(obj,~,ps_x,~) %(obj,t,x,u)
            guard_s2f = [ps_x(9);ps_x(11)];
            if ps_x(3) == obj.rest_l1 && ps_x(5) == obj.rest_l2
                guard_s2f = [0;0];
            end
        end
        
        function guard_f2s = flight2stanceGuard(obj,~,~,~) %(obj,t,x,u)
            if obj.xfoot1 > obj.xfoot2 %&& pf_x(4) == atan2(pf_x(2),obj.xfoot1-pf_x(1))
                guard_f2s = obj.yfoot1;
            elseif obj.xfoot1 < obj.xfoot2 %&& pf_x(6) == atan2(pf_x(2),obj.xfoot2-pf_x(1))
                guard_f2s = obj.yfoot2;
            end
        end
        
        function [pd_x,mode,status,dpd_x]=single2double(~,mode,~,ps_x,~) %(obj,mode,t,xm,u)
            pd_x=[ps_x(5)-ps_x(1)*sin(ps_x(2));...
                ps_x(1)*cos(ps_x(2));...
                -ps_x(3)*sin(ps_x(2))-ps_x(1)*ps_x(4)*cos(ps_x(2));...
                ps_x(3)*cos(ps_x(2))-ps_x(1)*ps_x(4)*sin(ps_x(2));];
            if(mode~=ps_mode)
                error('Incorrect mode');
            else
                mode=pd_mode;
            end
            status=(pd_x(3)<0);  % terminate if xdot < 0
            dpd_xdps_x=zeros(4,5);
            dpd_xdps_x(1,5)=1;
            dpd_xdps_x(1,1)=-sin(ps_x(2));
            dpd_xdps_x(1,2)=-ps_x(1)*cos(ps_x(2));
            dpd_xdps_x(1,5)=1;
            dpd_xdps_x(2,1)=cos(ps_x(2));
            dpd_xdps_x(2,2)=-ps_x(1)*sin(ps_x(2));
            dpd_xdps_x(2,6)=1;
            dpd_xdps_x(3,1)=-ps_x(4)*cos(ps_x(2));
            dpd_xdps_x(3,2)=-ps_x(3)*cos(ps_x(2))+ps_x(1)*ps_x(4)*sin(ps_x(2));
            dpd_xdps_x(3,3)=-sin(ps_x(2));
            dpd_xdps_x(3,4)=-ps_x(1)*cos(ps_x(2));
            dpd_xdps_x(4,1)=-ps_x(4)*sin(ps_x(2));
            dpd_xdps_x(4,2)=-ps_x(3)*sin(ps_x(2))-ps_x(1)*ps_x(4)*cos(ps_x(2));
            dpd_xdps_x(4,3)=cos(ps_x(2));
            dpd_xdps_x(4,4)=-ps_x(1)*sin(ps_x(2));
            dpd_x=[zeros(4,2) dpd_xdps_x zeros(4,1)];
        end
        
        function [ps_x,mode,status,dps_x]=double2single(~,mode,~,pd_x,u) %(obj,mode,t,xm,u)
            if(mode~=pd_mode)
                error('Incorrect mode');
            else
                mode=ps_mode;
            end
            theta=u;
            r= pd_x(2)/cos(theta); % = obj.rest_l1 because x(2)-obj.rest_l1*cos(u) = 0
            ps_x=[r;...
                theta;...
                -pd_x(3)*sin(theta)+pd_x(4)*cos(theta);...
                -(pd_x(3)*cos(theta)+pd_x(4)*sin(theta))/r;...
                pd_x(1)+r*sin(theta);0;0;0;0;0;0;0]; %pd_x(2)*tan(theta)];
            status=0;
            if nargout>3
                dps_xdpd_x=zeros(5,4);
                dps_xdu=zeros(5,1);
                dps_xdpd_x(1,2)=1/cos(theta);
                dps_xdu(1,1)=(pd_x(2))*sin(theta)/cos(theta)^2;
                %dps_xdpd_x(1,6)=-1/cos(theta);
                dps_xdu(2,1)=1;
                dps_xdpd_x(3,3)=-sin(theta);
                dps_xdpd_x(3,4)=cos(theta);
                dps_xdu(3,1)=-pd_x(3)*cos(theta)-pd_x(4)*sin(theta);
                dps_xdpd_x(4,2)=(pd_x(3)*cos(theta)^2+pd_x(4)*sin(theta)*cos(theta))/(pd_x(2))^2;
                dps_xdpd_x(4,3)=-cos(theta)^2/(pd_x(2));
                dps_xdpd_x(4,4)=-sin(theta)*cos(theta)/(pd_x(2));
                dps_xdu(4,1)=-(-pd_x(3)*sin(2*theta)+pd_x(4)*cos(2*theta))/(pd_x(2));
                %dps_xdpd_x(4,6)=-dps_xdpd_x(4,2);
                dps_xdpd_x(5,1)=1;
                dps_xdpd_x(5,2)=tan(theta);
                %error('still need to update this last one:');
                dps_xdu(5,1)=pd_x(2)/cos(theta)^2;
                dps_x=[zeros(5,2) dps_xdpd_x dps_xdu];
            end
        end
        
        function [pf_x,mode,status,dpf_x]=stance2flight(~,mode,~,ps_x,~) %(obj,mode,t,xm,u)
            pf_x=[ps_x(5)-ps_x(1)*sin(ps_x(2));...
                ps_x(1)*cos(ps_x(2));...
                -ps_x(3)*sin(ps_x(2))-ps_x(1)*ps_x(4)*cos(ps_x(2));...
                ps_x(3)*cos(ps_x(2))-ps_x(1)*ps_x(4)*sin(ps_x(2));];
            if(mode~=ps_mode)
                error('Incorrect mode');
            else
                mode=pf_mode;
            end
            status=(pf_x(3)<0);  % terminate if xdot < 0
            dpf_xdps_x=zeros(4,5);
            dpf_xdps_x(1,5)=1;
            dpf_xdps_x(1,1)=-sin(ps_x(2));
            dpf_xdps_x(1,2)=-ps_x(1)*cos(ps_x(2));
            dpf_xdps_x(1,5)=1;
            dpf_xdps_x(2,1)=cos(ps_x(2));
            dpf_xdps_x(2,2)=-ps_x(1)*sin(ps_x(2));
            dpf_xdps_x(2,6)=1;
            dpf_xdps_x(3,1)=-ps_x(4)*cos(ps_x(2));
            dpf_xdps_x(3,2)=-ps_x(3)*cos(ps_x(2))+ps_x(1)*ps_x(4)*sin(ps_x(2));
            dpf_xdps_x(3,3)=-sin(ps_x(2));
            dpf_xdps_x(3,4)=-ps_x(1)*cos(ps_x(2));
            dpf_xdps_x(4,1)=-ps_x(4)*sin(ps_x(2));
            dpf_xdps_x(4,2)=-ps_x(3)*sin(ps_x(2))-ps_x(1)*ps_x(4)*cos(ps_x(2));
            dpf_xdps_x(4,3)=cos(ps_x(2));
            dpf_xdps_x(4,4)=-ps_x(1)*sin(ps_x(2));
            dpf_x=[zeros(4,2) dpf_xdps_x zeros(4,1)];
        end
        
        function [ps_x,mode,status,dps_x]=flight2stance(~,mode,~,pf_x,u) %(obj,mode,t,xm,u)
            if(mode~=pf_mode)
                error('Incorrect mode');
            else
                mode=ps_mode;
            end
            theta=u;
            r= pf_x(2)/cos(theta); % = obj.rest_l1 because x(2)-obj.rest_l1*cos(u) = 0
            ps_x=[r;...
                theta;...
                -pf_x(3)*sin(theta)+pf_x(4)*cos(theta);...
                -(pf_x(3)*cos(theta)+pf_x(4)*sin(theta))/r;...
                pf_x(1)+r*sin(theta);0;0;0;0;0;0;0]; %pf_x(2)*tan(theta)];
            status=0;
            if nargout>3
                dps_xdpf_x=zeros(5,4);
                dps_xdu=zeros(5,1);
                dps_xdpf_x(1,2)=1/cos(theta);
                dps_xdu(1,1)=(pf_x(2))*sin(theta)/cos(theta)^2;
                %dps_xdpf_x(1,6)=-1/cos(theta);
                dps_xdu(2,1)=1;
                dps_xdpf_x(3,3)=-sin(theta);
                dps_xdpf_x(3,4)=cos(theta);
                dps_xdu(3,1)=-pf_x(3)*cos(theta)-pf_x(4)*sin(theta);
                dps_xdpf_x(4,2)=(pf_x(3)*cos(theta)^2+pf_x(4)*sin(theta)*cos(theta))/(pf_x(2))^2;
                dps_xdpf_x(4,3)=-cos(theta)^2/(pf_x(2));
                dps_xdpf_x(4,4)=-sin(theta)*cos(theta)/(pf_x(2));
                dps_xdu(4,1)=-(-pf_x(3)*sin(2*theta)+pf_x(4)*cos(2*theta))/(pf_x(2));
                %dps_xdpf_x(4,6)=-dps_xdpf_x(4,2);
                dps_xdpf_x(5,1)=1;
                dps_xdpf_x(5,2)=tan(theta);
                %error('still need to update this last one:');
                dps_xdu(5,1)=pf_x(2)/cos(theta)^2;
                dps_x=[zeros(5,2) dps_xdpf_x dps_xdu];
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