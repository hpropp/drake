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
            
            if obj.alpha0 >= pi/3 % the model is walking
                % singleSupport mode (one leg on the ground, one leg in the air)
                pSingleSupport=BipedalSLIPSingleSupport(obj);
                [obj, ps_mode]=obj.addMode(pSingleSupport);
                
                % doubleSupport mode, where both legs are on the ground
                pDoubleSupport=BipedalSLIPDoubleSupport(obj);
                [obj, pd_mode]=obj.addMode(pDoubleSupport);
                
                obj=addTransition(obj,ps_mode,@single2doubleGuard,@single2double,true,true,pd_mode);
                obj=addTransition(obj,pd_mode,@double2SingleGuard,@double2single,true,true,ps_mode);
                
                obj.yfoot1 = 0;
                obj.xfoot1 = 0.25;
            elseif obj.alpha0 < pi/3 % the model is running
                % singleSupport mode (one leg on the ground, one leg in the air)
                pSingleSupport=BipedalSLIPSingleSupport(obj);
                [obj, ps_mode]=obj.addMode(pSingleSupport);
                
                % flight mode (both legs in the air)
                pFlight=BipedalSLIPFlight(obj);
                [obj, pf_mode]=obj.addMode(pFlight);
                
                obj=addTransition(obj,ps_mode,@stance2flightGuard,@stance2flight,true,true,pf_mode);
                obj=addTransition(obj,pf_mode,@flight2stanceGuard,@flight2stance,true,true,ps_mode);
                
                obj.yfoot1 = 0;
                obj.xfoot1 = 0.25;
            end
        end
        
        function guard_s2d = single2doubleGuard(obj,~,ps_x,~) %(obj,t,x,u)
            if obj.yfoot1 == 0 %obj.yfoot2~=0
                theta1 = atan2(ps_x(2),obj.xfoot1-ps_x(1));
                theta2 = obj.alpha0;
                if theta1 == atan2(ps_x(2),obj.xfoot1-ps_x(1))
                    if theta2 == atan2(ps_x(2),obj.xfoot2-ps_x(1))
                        obj.yfoot1 = 0;
                        obj.yfoot2 = 0;
                    end
                end
            elseif obj.yfoot2 == 0 %obj.yfoot1~=0
                theta1 = obj.alpha0;
                theta2 = atan2(ps_x(2),obj.xfoot2-ps_x(1));
                if theta1 == atan2(ps_x(2),obj.xfoot1-ps_x(1))
                    if theta2 == atan2(ps_x(2),obj.xfoot2-ps_x(1))
                        obj.yfoot1 = 0;
                        obj.yfoot2 = 0;
                    end
                end
            end
            
            guard_s2d = [obj.yfoot1;obj.yfoot2];
        end
        
        function guard_d2s = double2SingleGuard(obj,~,pd_x,~) %(obj,t,x,u)
            r1 = sqrt((x(1)-obj.xfoot1)^2+x(2)^2);
            r2 = sqrt((x(1)-obj.xfoot2)^2+x(2)^2);
            r1dot = (((pd_x(1)-obj.xfoot1)*pd_x(3))+(pd_x(2)*pd_x(4)))/sqrt((pd_x(1)^2)-(2*obj.xfoot1*pd_x(1))+(pd_x(2)^2)+(obj.xfoot1^2));
            r2dot = (((pd_x(1)-obj.xfoot2)*pd_x(3))+(pd_x(2)*pd_x(4)))/sqrt((pd_x(1)^2)-(2*obj.xfoot2*pd_x(1))+(pd_x(2)^2)+(obj.xfoot2^2));
            
            if obj.xfoot1 > obj.xfoot2 && r1 == obj.rest_l1
                guard_d2s = r2dot;
            elseif obj.xfoot1 < obj.xfoot2 && r2 == obj.rest_l2
                guard_d2s = r1dot;
            end
        end
        
        function guard_s2f = stance2flightGuard(obj,~,ps_x,~) %(obj,t,x,u)
            if obj.yfoot1 == 0
                r1 = sqrt((ps_x(1)-obj.xfoot1)^2+ps_x(2)^2);
                r2 = obj.rest_l2;
                r1dot = (((ps_x(1)-obj.xfoot1)*ps_x(3))+(ps_x(2)*ps_x(4)))/sqrt((ps_x(1)^2)-(2*obj.xfoot1*ps_x(1))+(ps_x(2)^2)+(obj.xfoot1^2));
                r2dot = 0;
            elseif obj.yfoot2 == 0
                r1 = obj.rest_l1;
                r2 = sqrt((ps_x(1)-obj.xfoot2)^2+ps_x(2)^2);
                r1dot = 0;
                r2dot = (((ps_x(1)-obj.xfoot2)*ps_x(3))+(ps_x(2)*ps_x(4)))/sqrt((ps_x(1)^2)-(2*obj.xfoot2*ps_x(1))+(ps_x(2)^2)+(obj.xfoot2^2));
            end
            
            guard_s2f = [r1dot;r2dot];
            if r1 == obj.rest_l1 && r2 == obj.rest_l2
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
        
        function [pd_x,mode,status,dpd_x]=single2double(obj,mode,~,ps_x,~) %(obj,mode,t,x,u)
            pd_x=ps_x(1:4);
            if(mode~=ps_mode)
                error('Incorrect mode');
            else
                mode=pd_mode;
            end
            status=(pd_x(3)<0);  % terminate if xdot < 0
            
            r1 = sqrt((x(1)-obj.xfoot1)^2+x(2)^2);
            r2 = sqrt((x(1)-obj.xfoot2)^2+x(2)^2);
            theta1 = atan2(x(2),obj.xfoot1-x(1));
            theta2 = atan2(x(2),obj.xfoot2-x(1));
            
            obj.xfoot1 = x+(r1*cos(theta1)); % x position of r1
            obj.xfoot2 = x+(r2*cos(theta2)); % x position of r2
            obj.yfoot1 = 0; % y position of r1
            obj.yfoot2 = 0; % y position of r2
            
            F1 = [obj.k*(r1-obj.rest_l1)*cos(theta1);-obj.k*(y(3)-obj.rest_l1)*sin(theta1)];
            F2 = [obj.k*(r2-obj.rest_l2)*cos(theta2);-obj.k*(y(5)-obj.rest_l2)*sin(theta2)];
            F3 = [0;-obj.m_hip*obj.g];
            dpd_x=[ps_x(3:4) (F1+F2+F3)/obj.m_hip];
        end
        
        function [ps_x,mode,status,dps_x]=double2single(obj,mode,~,pd_x,~) %(obj,mode,t,x,u)
            ps_x=pd_x(1:4);
            if(mode~=pd_mode)
                error('Incorrect mode');
            else
                mode=ps_mode;
            end
            status=0;
            if pd_x(5) == obj.rest_l2
                obj.xfoot1 = y(1)+((y(2)*cos(y(4)))/sin(y(4))); % x position of r1
                obj.yfoot1 = 0; % y position of r1
                
                F1 = [obj.k*(y(3)-obj.rest_l1)*cos(y(4));-obj.k*(y(3)-obj.rest_l1)*sin(y(4))];
                F3 = [0;-obj.m_hip*obj.g];
                dps_x=[pd_x(3:4) (F1+F3)/obj.m_hip];
                
            elseif pd_x(3) == obj.rest_l1
                obj.xfoot2 = y(1)+((y(2)*cos(y(6)))/sin(y(6))); % x position of r2
                obj.yfoot2 = 0; % y position of r2
                
                F2 = [obj.k*(y(5)-obj.rest_l2)*cos(y(6));-obj.k*(y(5)-obj.rest_l2)*sin(y(6))];
                F3 = [0;-obj.m_hip*obj.g];
                dps_x=[pd_x(3:4) (F2+F3)/obj.m_hip];
            end
        end
        
        function [pf_x,mode,status,dpf_x]=stance2flight(~,mode,~,ps_x,~) %(obj,mode,t,x,u)
            pf_x=ps_x(1:4);
            if(mode~=ps_mode)
                error('Incorrect mode');
            else
                mode=pf_mode;
            end
            status=(pf_x(3)<0);  % terminate if xdot < 0
            
            obj.xfoot1 = y(1)+(y(3)*cos(y(4))); % x position of r1
            obj.yfoot1 = y(2)-sqrt((y(3)^2)-((y(1)-obj.xfoot1)^2)); % y position of r1
            obj.xfoot2 = y(1)+(y(5)*cos(y(6))); % x position of r2
            obj.yfoot2 = y(2)-sqrt((y(5)^2)-((y(1)-obj.xfoot2)^2)); % y position of r2
            
            F3 = [0;-obj.m_hip*obj.g];
            dpf_x=[ps_x(3:4) (F3)/obj.m_hip];
        end
        
        function [ps_x,mode,status,dps_x]=flight2stance(obj,mode,~,pf_x,~) %(obj,mode,t,x,u)
            ps_x=pf_x(1:4);
            if(mode~=pf_mode)
                error('Incorrect mode');
            else
                mode=ps_mode;
            end
            status=0;
            if obj.xfoot1 > obj.xfoot2
                obj.xfoot1 = y(1)+((y(2)*cos(y(4)))/sin(y(4))); % x position of r1
                obj.yfoot1 = 0; % y position of r1
                
                F1 = [obj.k*(y(3)-obj.rest_l1)*cos(y(4));-obj.k*(y(3)-obj.rest_l1)*sin(y(4))];
                F3 = [0;-obj.m_hip*obj.g];
                dps_x=[pf_x(3:4) (F1+F3)/obj.m_hip];
                
            elseif obj.xfoot1 < obj.xfoot2
                obj.xfoot2 = y(1)+((y(2)*cos(y(6)))/sin(y(6))); % x position of r2
                obj.yfoot2 = 0; % y position of r2
                
                F2 = [obj.k*(y(5)-obj.rest_l2)*cos(y(6));-obj.k*(y(5)-obj.rest_l2)*sin(y(6))];
                F3 = [0;-obj.m_hip*obj.g];
                dps_x=[pf_x(3:4) (F2+F3)/obj.m_hip];
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