classdef BipedalSLIP < HybridDrakeSystem
    % a script combining all of the other classes
    % the control input (aka 'u') is the 'angle of attack'
    
    properties
        r_rest = 1; % rest length of leg springs (m)
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
                obj.xfoot1 = 0.35;
                obj.yfoot1 = 0;
                
                % singleSupport mode (one leg on the ground, one leg in the air)
                pr1SingleSupport=BipedalSLIPSingleSupportLeg1(obj);
                [obj, ps_r1_mode]=obj.addMode(pr1SingleSupport);
                pr2SingleSupport=BipedalSLIPSingleSupportLeg2(obj);
                [obj, ps_r2_mode]=obj.addMode(pr2SingleSupport);
                
                % doubleSupport mode, where both legs are on the ground
                pDoubleSupport=BipedalSLIPDoubleSupport(obj);
                [obj, pd_mode]=obj.addMode(pDoubleSupport);
                
                obj=addTransition(obj,ps_r1_mode,@r2PenetrateGuard,@r1single2double,true,true,pd_mode);
                obj=addTransition(obj,pd_mode,andGuards(@r1restlengthGuard,@r1legextensionGuard),@double2r2single,true,true,ps_r2_mode);
                obj=addTransition(obj,ps_r2_mode,@r1PenetrateGuard,@r2single2double,true,true,pd_mode);
                obj=addTransition(obj,pd_mode,andGuards(@r2restlengthGuard,@r2legextensionGuard),@double2r1single,true,true,ps_r1_mode);
                
            else % the model is running
                obj.xfoot1 = 0.35;
                obj.yfoot1 = 0;
                
                % singleSupport mode (one leg on the ground, one leg in the air)
                pr1SingleSupport=BipedalSLIPSingleSupportLeg1(obj);
                [obj, ps_r1_mode]=obj.addMode(pr1SingleSupport);
                pr2SingleSupport=BipedalSLIPSingleSupportLeg2(obj);
                [obj, ps_r2_mode]=obj.addMode(pr2SingleSupport);
                
                % flight mode (both legs in the air)
                pFlight=BipedalSLIPFlight(obj);
                [obj, pf_mode]=obj.addMode(pFlight);
                
                obj=addTransition(obj,ps_r1_mode,andGuards(@r1restlengthGuard,@r1legextensionGuard),@r1stance2flight,true,true,pf_mode);
                obj=addTransition(obj,pf_mode,@r2PenetrateGuard,@flight2r2stance,true,true,ps_r2_mode);
                obj=addTransition(obj,ps_r2_mode,andGuards(@r2restlengthGuard,@r2legextensionGuard),@r2stance2flight,true,true,pf_mode);
                obj=addTransition(obj,pf_mode,@r1PenetrateGuard,@flight2r1stance,true,true,ps_r1_mode);
            end
        end
        
        function guard_s2d = r1PenetrateGuard(obj,t,ps_x,u) %(obj,t,x,u)
            
            guard_s2d = ps_x(2)-(obj.r_rest*sin(u));
            
            if obj.yfoot1 < 1e-6
                obj.xfoot1 = ps_x(3);
                obj.xfoot2 = ps_x(1)+(obj.r_rest*cos(u));

            elseif obj.yfoot2 < 1e-6
                obj.xfoot1 = ps_x(1)+(obj.r_rest*cos(u));
                obj.xfoot2 = ps_x(3);
            end
        end
        
        function guard_d2s = r1restlengthGuard(obj,~,pd_x,~) %(obj,t,x,u)
            if pd_x(3) < pd_x(5)
                r1 = sqrt((pd_x(1)-pd_x(3))^2+pd_x(2)^2);
                r1dot = (((pd_x(1)-pd_x(3))*pd_x(7))+(pd_x(2)*pd_x(8)))/sqrt((pd_x(1)^2)-(2*pd_x(3)*pd_x(1))+(pd_x(2)^2)+(pd_x(3)^2));
                guard_d2s = r1-obj.r_rest;
%                 if abs(r1-obj.r_rest) < 0.2
%                     guard_d2s = 0;
%                 end
            end
            if pd_x(3) > pd_x(5)
                r2 = sqrt((pd_x(1)-pd_x(5))^2+pd_x(2)^2);
                guard_d2s = r2-obj.r_rest;
%                 if abs(r2-obj.r_rest) < 0.2
%                     guard_d2s = 0;
%                 end
            end
        end
        
        function guard_d2s = r1legextensionGuard(obj,~,pd_x,~) %(obj,t,x,u)
            if pd_x(3) < pd_x(5)
                r1 = sqrt((pd_x(1)-pd_x(3))^2+pd_x(2)^2);
                r1dot = (((pd_x(1)-pd_x(3))*pd_x(7))+(pd_x(2)*pd_x(8)))/sqrt((pd_x(1)^2)-(2*pd_x(3)*pd_x(1))+(pd_x(2)^2)+(pd_x(3)^2));
                guard_d2s = r1-obj.r_rest;
%                 if abs(r1-obj.r_rest) < 0.2
%                     guard_d2s = 0;
%                 end
            end
            if pd_x(3) > pd_x(5)
                r2 = sqrt((pd_x(1)-pd_x(5))^2+pd_x(2)^2);
                guard_d2s = r2-obj.r_rest;
%                 if abs(r2-obj.r_rest) < 0.2
%                     guard_d2s = 0;
%                 end
            end
        end
        
        function guard_s2d = r2PenetrateGuard(obj,t,ps_x,u) %(obj,t,x,u)
            
            guard_s2d = ps_x(2)-(obj.r_rest*sin(u));
            
            if obj.yfoot1 < 1e-6
                obj.xfoot1 = ps_x(3);
                obj.xfoot2 = ps_x(1)+(obj.r_rest*cos(u));

            elseif obj.yfoot2 < 1e-6
                obj.xfoot1 = ps_x(1)+(obj.r_rest*cos(u));
                obj.xfoot2 = ps_x(3);
            end
        end
        
        function guard_d2s = r2restlengthGuard(obj,~,pd_x,~) %(obj,t,x,u)
            if pd_x(3) < pd_x(5)
                r1 = sqrt((pd_x(1)-pd_x(3))^2+pd_x(2)^2);
                r1dot = (((pd_x(1)-pd_x(3))*pd_x(7))+(pd_x(2)*pd_x(8)))/sqrt((pd_x(1)^2)-(2*pd_x(3)*pd_x(1))+(pd_x(2)^2)+(pd_x(3)^2));
                guard_d2s = r1-obj.r_rest;
%                 if abs(r1-obj.r_rest) < 0.2
%                     guard_d2s = 0;
%                 end
            end
            if pd_x(3) > pd_x(5)
                r2 = sqrt((pd_x(1)-pd_x(5))^2+pd_x(2)^2);
                guard_d2s = r2-obj.r_rest;
%                 if abs(r2-obj.r_rest) < 0.2
%                     guard_d2s = 0;
%                 end
            end
        end
        
        function guard_d2s = r2legextensionGuard(obj,~,pd_x,~) %(obj,t,x,u)
            if pd_x(3) < pd_x(5)
                r1 = sqrt((pd_x(1)-pd_x(3))^2+pd_x(2)^2);
                r1dot = (((pd_x(1)-pd_x(3))*pd_x(7))+(pd_x(2)*pd_x(8)))/sqrt((pd_x(1)^2)-(2*pd_x(3)*pd_x(1))+(pd_x(2)^2)+(pd_x(3)^2));
                guard_d2s = r1-obj.r_rest;
%                 if abs(r1-obj.r_rest) < 0.2
%                     guard_d2s = 0;
%                 end
            end
            if pd_x(3) > pd_x(5)
                r2 = sqrt((pd_x(1)-pd_x(5))^2+pd_x(2)^2);
                guard_d2s = r2-obj.r_rest;
%                 if abs(r2-obj.r_rest) < 0.2
%                     guard_d2s = 0;
%                 end
            end
        end
        
        function [pd_x,mode,status]=r1single2double(obj,mode,~,ps_x,u) %(obj,mode,t,x,u)
            if obj.yfoot1 < 1e-6
                pd_x=[ps_x(1:2);ps_x(3);0;ps_x(1)+(obj.r_rest*cos(u));0;ps_x(5:6)];
                obj.xfoot1 = ps_x(3);
                obj.xfoot2 = ps_x(1)+(obj.r_rest*cos(u));
            elseif obj.yfoot2 < 1e-6
                pd_x=[ps_x(1:2);ps_x(1)+(obj.r_rest*cos(u));0;ps_x(3);0;ps_x(5:6)];
                obj.xfoot1 = ps_x(1)+(obj.r_rest*cos(u));
                obj.xfoot2 = ps_x(3);
            end
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=2;
            end
            status=(pd_x(7)<0);  % terminate if xdot < 0
        end
        
        function [ps_x,mode,status]=double2r2single(obj,mode,~,pd_x,u) %(obj,mode,t,x,u)
            if pd_x(3) < pd_x(5)
                ps_x=[pd_x(1:2);pd_x(5:8)];
                obj.xfoot1 = ps_x(3); % x position of r1
                obj.yfoot1 = ps_x(4); % y position of r1
                obj.xfoot2 = pd_x(1)+((pd_x(2)*cos(u))/sin(u)); % x position of r2
                obj.yfoot2 = 0; % y position of r2
            end
            if pd_x(3) > pd_x(5)
                ps_x=[pd_x(1:4);pd_x(7:8)];
                obj.xfoot1 = pd_x(1)+((pd_x(2)*cos(u))/sin(u)); % x position of r1
                obj.yfoot1 = 0; % y position of r1
                obj.xfoot2 = ps_x(3); % x position of r2
                obj.yfoot2 = ps_x(4); % y position of r2
            end
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=1;
            end
            status=0;
        end
        
        function [pd_x,mode,status]=r2single2double(obj,mode,~,ps_x,u) %(obj,mode,t,x,u)
            if obj.yfoot1 < 1e-6
                pd_x=[ps_x(1:2);ps_x(3);0;ps_x(1)+(obj.r_rest*cos(u));0;ps_x(5:6)];
                obj.xfoot1 = ps_x(3);
                obj.xfoot2 = ps_x(1)+(obj.r_rest*cos(u));
            elseif obj.yfoot2 < 1e-6
                pd_x=[ps_x(1:2);ps_x(1)+(obj.r_rest*cos(u));0;ps_x(3);0;ps_x(5:6)];
                obj.xfoot1 = ps_x(1)+(obj.r_rest*cos(u));
                obj.xfoot2 = ps_x(3);
            end
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=2;
            end
            status=(pd_x(7)<0);  % terminate if xdot < 0
        end
        
        function [ps_x,mode,status]=double2r1single(obj,mode,~,pd_x,u) %(obj,mode,t,x,u)
            if pd_x(3) < pd_x(5)
                ps_x=[pd_x(1:2);pd_x(5:8)];
                obj.xfoot1 = ps_x(3); % x position of r1
                obj.yfoot1 = ps_x(4); % y position of r1
                obj.xfoot2 = pd_x(1)+((pd_x(2)*cos(u))/sin(u)); % x position of r2
                obj.yfoot2 = 0; % y position of r2
            end
            if pd_x(3) > pd_x(5)
                ps_x=[pd_x(1:4);pd_x(7:8)];
                obj.xfoot1 = pd_x(1)+((pd_x(2)*cos(u))/sin(u)); % x position of r1
                obj.yfoot1 = 0; % y position of r1
                obj.xfoot2 = ps_x(3); % x position of r2
                obj.yfoot2 = ps_x(4); % y position of r2
            end
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=1;
            end
            status=0;
        end
        
        function [pf_x,mode,status]=r1stance2flight(obj,mode,~,ps_x,~) %(obj,mode,t,x,u)
            pf_x=[ps_x(1:2);ps_x(5:6)];
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=2;
            end
            status=(pf_x(3)<0);  % terminate if xdot < 0
%             r1 = obj.r_rest;
%             r2 = obj.r_rest;
%             theta1 = atan2(sqrt((r1^2)-((ps_x(1)-obj.xfoot1)^2)),obj.xfoot1-ps_x(1));
%             theta2 = atan2(sqrt((r2^2)-((ps_x(1)-obj.xfoot2)^2)),obj.xfoot2-ps_x(1));
%             
%             obj.xfoot1 = ps_x(1)+(r1*cos(theta1)); % x position of r1
%             obj.xfoot2 = ps_x(1)+(r2*cos(theta2)); % x position of r2
        end
        
        function [ps_x,mode,status]=flight2r1stance(obj,mode,~,pf_x,~) %(obj,mode,t,x,u)
            ps_x=pf_x(1:4);
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=1;
            end
            status=0;
            if obj.xfoot1 > obj.xfoot2 %obj.xfoot2==0
                theta1 = obj.alpha0;
                obj.xfoot1 = pf_x(1)+((pf_x(2)*cos(theta1))/sin(theta1)); % x position of r1
                obj.yfoot1 = 0; % y position of r1
            elseif obj.xfoot1 < obj.xfoot2 %obj.xfoot1==0
                theta2 = obj.alpha0;
                obj.xfoot2 = pf_x(1)+((pf_x(2)*cos(theta2))/sin(theta2)); % x position of r2
                obj.yfoot2 = 0; % y position of r2
            end
        end
        
        function [pf_x,mode,status]=r2stance2flight(obj,mode,~,ps_x,~) %(obj,mode,t,x,u)
            pf_x=[ps_x(1:2);ps_x(5:6)];
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=2;
            end
            status=(pf_x(3)<0);  % terminate if xdot < 0
%             r1 = obj.r_rest;
%             r2 = obj.r_rest;
%             theta1 = atan2(sqrt((r1^2)-((ps_x(1)-obj.xfoot1)^2)),obj.xfoot1-ps_x(1));
%             theta2 = atan2(sqrt((r2^2)-((ps_x(1)-obj.xfoot2)^2)),obj.xfoot2-ps_x(1));
%             
%             obj.xfoot1 = ps_x(1)+(r1*cos(theta1)); % x position of r1
%             obj.xfoot2 = ps_x(1)+(r2*cos(theta2)); % x position of r2
        end
        
        function [ps_x,mode,status]=flight2r2stance(obj,mode,~,pf_x,~) %(obj,mode,t,x,u)
            ps_x=pf_x(1:4);
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=1;
            end
            status=0;
            if obj.xfoot1 > obj.xfoot2 %obj.xfoot2==0
                theta1 = obj.alpha0;
                obj.xfoot1 = pf_x(1)+((pf_x(2)*cos(theta1))/sin(theta1)); % x position of r1
                obj.yfoot1 = 0; % y position of r1
            elseif obj.xfoot1 < obj.xfoot2 %obj.xfoot1==0
                theta2 = obj.alpha0;
                obj.xfoot2 = pf_x(1)+((pf_x(2)*cos(theta2))/sin(theta2)); % x position of r2
                obj.yfoot2 = 0; % y position of r2
            end
        end
        
        function theta=angleOfAttackControl(obj,yd,y0,xdot0)
            aoaControl=@(theta) obj.apex2apex(theta,y0,xdot0)-yd;
            theta=fzero(aoaControl,pi/8);
        end
    end
    
    methods (Static)
        function run(u)
            r = BipedalSLIP();
            vis = BipedalSLIPVisualizer(r);
            
            aoa_command = setOutputFrame(ConstantTrajectory(u),getInputFrame(r));
            ytraj = simulate(cascade(aoa_command,r),[0 11]);
            vis.playback(ytraj,struct('slider','true'));
        end
    end
end