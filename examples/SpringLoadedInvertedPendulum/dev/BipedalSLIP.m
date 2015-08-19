classdef BipedalSLIP < HybridDrakeSystem
    % a script combining all of the other bipedalSLIP classes
    % the control input (aka 'u') is the 'angle of attack'
    
    properties
        r_rest = 1; % rest length of leg springs (m)
        m_hip = 1; % mass (kg)
        k = 100; % stiffness spring coefficient (aka elastic restoring force)
        g = 9.81; % gravity (m/s^2)
        alpha0 = pi/3; % the fixed leg orientation (for swing phases) with respect to gravity aka the angle of attack
        
        r1atground = [];
        r2atground = [];
    end
    
    methods
        function obj = BipedalSLIP()
            obj = obj@HybridDrakeSystem(...
                1, ... % number of inputs
                12); % number of outputs
            
            obj = setInputFrame(obj,CoordinateFrame('SLIPInput',1,'u',{'alpha0'})); % alpha0 is the angle of attack
            obj = setOutputFrame(obj,CoordinateFrame('SLIPOutput',12,'y',{'x','y','r1','theta1','r2','theta2','xdot','ydot','r1dot','theta1dot','r2dot','theta2dot'}));
            
            if obj.alpha0 >= pi/6 % the model is walking
                obj.r1atground = true;
                obj.r2atground = false;
                
                % singleSupport mode (one leg on the ground, one leg in the air)
                pr1SingleSupport=BipedalSLIPSingleSupport(obj);
                [obj, ps_r1_mode]=obj.addMode(pr1SingleSupport); % mode 1
                pr2SingleSupport=BipedalSLIPSingleSupport(obj);
                [obj, ps_r2_mode]=obj.addMode(pr2SingleSupport); % mode 2
                
                % doubleSupport mode, where both legs are on the ground
                pDoubleSupport=BipedalSLIPDoubleSupport(obj);
                [obj, pd_mode]=obj.addMode(pDoubleSupport); % mode 3
                
                obj=addTransition(obj,ps_r1_mode,@r2PenetrateGuard,@r1single2double,true,true,pd_mode);
                obj=addTransition(obj,pd_mode,andGuards(obj,@r1restlengthGuard,@r1legextensionwalkGuard),@double2r2single,true,true,ps_r2_mode);
                obj=addTransition(obj,ps_r2_mode,@r1PenetrateGuard,@r2single2double,true,true,pd_mode);
                obj=addTransition(obj,pd_mode,andGuards(obj,@r2restlengthwalkGuard,@r2legextensionwalkGuard),@double2r1single,true,true,ps_r1_mode);
                
            else % the model is running
                obj.r1atground = true;
                obj.r2atground = false;
                
                % singleSupport mode (one leg on the ground, one leg in the air)
                pr1SingleSupport=BipedalSLIPSingleSupportLeg1(obj); 
                [obj, ps_r1_mode]=obj.addMode(pr1SingleSupport); % mode 1
                pr2SingleSupport=BipedalSLIPSingleSupportLeg2(obj);
                [obj, ps_r2_mode]=obj.addMode(pr2SingleSupport); % mode 2
                
                % flight mode (both legs in the air)
                pFlight=BipedalSLIPFlight(obj);
                [obj, pf_mode]=obj.addMode(pFlight); % mode 3
                
                obj=addTransition(obj,ps_r1_mode,andGuards(obj,@r1restlengthGuard,@r1legextensionrunGuard),@r1stance2flight,true,true,pf_mode);
                obj=addTransition(obj,pf_mode,@r2PenetrateGuard,@flight2r2stance,true,true,ps_r2_mode);
                obj=addTransition(obj,ps_r2_mode,andGuards(obj,@r2restlengthrunGuard,@r2legextensionrunGuard),@r2stance2flight,true,true,pf_mode);
                obj=addTransition(obj,pf_mode,@r1PenetrateGuard,@flight2r1stance,true,true,ps_r1_mode);
            end
        end
        
        function r1pguard = r1PenetrateGuard(obj,t,ps2_x,u) %(obj,t,x,u)
            if ps2_x(3) < ps2_x(1)+(obj.r_rest*cos(u)) % xfoot1 > xfoot2
                r1pguard = ps2_x(2)-(obj.r_rest*sin(u));
            end
        end
        
        function r2pguard = r2PenetrateGuard(obj,t,ps1_x,u) %(obj,t,x,u)
            if ps1_x(3) < ps1_x(1)+(obj.r_rest*cos(u)) % xfoot1 < xfoot2
                r2pguard = ps1_x(2)-(obj.r_rest*sin(u));
            end
        end
        
        function r1restlguard = r1restlengthGuard(obj,t,pds1_x,~) %(obj,t,x,u)
            r1 = sqrt((pds1_x(1)-pds1_x(3))^2+pds1_x(2)^2);
            r1restlguard = -(r1-obj.r_rest); 
        end
        
        function r2restlguard = r2restlengthwalkGuard(obj,t,pd_x,~) %(obj,t,x,u)
            r2 = sqrt((pd_x(1)-pd_x(5))^2+pd_x(2)^2);
            r2restlguard = -(r2-obj.r_rest);
        end
        
        function r2restlguard = r2restlengthrunGuard(obj,t,ps2_x,~) %(obj,t,x,u)
            r2 = sqrt((ps2_x(1)-ps2_x(3))^2+ps2_x(2)^2);
            r2restlguard = r2-obj.r_rest;
        end
        
        function r1legextendguard = r1legextensionwalkGuard(~,t,pd_x,~) %(obj,t,x,u)
            r1dot = (((pd_x(1)-pd_x(3))*pd_x(7))+(pd_x(2)*pd_x(8)))/sqrt((pd_x(1)^2)-(2*pd_x(3)*pd_x(1))+(pd_x(2)^2)+(pd_x(3)^2));
            r1legextendguard = -r1dot; 
        end
        
        function r1legextendguard = r1legextensionrunGuard(~,t,ps1_x,~) %(obj,t,x,u)
            r1dot = (((ps1_x(1)-ps1_x(3))*ps1_x(5))+(ps1_x(2)*ps1_x(6)))/sqrt((ps1_x(1)^2)-(2*ps1_x(3)*ps1_x(1))+(ps1_x(2)^2)+(ps1_x(3)^2));
            r1legextendguard = -r1dot;
        end
        
        function r2legextendguard = r2legextensionwalkGuard(~,t,pd_x,~) %(obj,t,x,u)
            r2dot = (((pd_x(1)-pd_x(5))*pd_x(7))+(pd_x(2)*pd_x(8)))/sqrt((pd_x(1)^2)-(2*pd_x(5)*pd_x(1))+(pd_x(2)^2)+(pd_x(5)^2));
            r2legextendguard = -r2dot;
        end
        
        function r2legextendguard = r2legextensionrunGuard(~,t,ps2_x,~) %(obj,t,x,u)
            r2dot = (((ps2_x(1)-ps2_x(3))*ps2_x(5))+(ps2_x(2)*ps2_x(6)))/sqrt((ps2_x(1)^2)-(2*ps2_x(3)*ps2_x(1))+(ps2_x(2)^2)+(ps2_x(3)^2));
            r2legextendguard = -r2dot;
        end
        
        function [pd_x,mode,status]=r1single2double(obj,mode,t,ps1_x,u) %(obj,mode,t,x,u)
            pd_x=[ps1_x(1:2);ps1_x(3:4);ps1_x(1)+(obj.r_rest*cos(u));0;ps1_x(5:6)];
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=3;
            end
            obj.r1atground = true;
            obj.r2atground = true;
            status=(pd_x(7)<0);  % terminate if xdot < 0
        end
        
        function [ps2_x,mode,status]=double2r2single(obj,mode,t,pd_x,~) %(obj,mode,t,x,u)
            ps2_x=[pd_x(1:2);pd_x(5:8)];
            if(mode~=3)
                error('Incorrect mode');
            else
                mode=2;
            end
            obj.r1atground = false;
            obj.r2atground = true;
            status=0;
        end
        
        function [pd_x,mode,status]=r2single2double(obj,mode,t,ps2_x,u) %(obj,mode,t,x,u)
            pd_x=[ps2_x(1:2);ps2_x(1)+(obj.r_rest*cos(u));0;ps2_x(3);0;ps2_x(5:6)];
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=3;
            end
            obj.r1atground = true;
            obj.r2atground = true;
            status=(pd_x(7)<0);  % terminate if xdot < 0
        end
        
        function [ps1_x,mode,status]=double2r1single(obj,mode,t,pd_x,~) %(obj,mode,t,x,u)
            ps1_x=[pd_x(1:4);pd_x(7:8)];
            if(mode~=3)
                error('Incorrect mode');
            else
                mode=1;
            end
            obj.r1atground = true;
            obj.r2atground = false;
            status=0;
        end
        
        function [pf_x,mode,status]=r1stance2flight(~,mode,~,ps1_x,~) %(obj,mode,t,x,u)
            pf_x=[ps1_x(1:2);ps1_x(5:6)];
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=3;
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
        
        function [ps2_x,mode,status]=flight2r2stance(obj,mode,~,pf_x,u) %(obj,mode,t,x,u)
            ps2_x=pf_x(1:4);
            if(mode~=3)
                error('Incorrect mode');
            else
                mode=2;
            end
            status=0;
            if obj.xfoot1 > obj.xfoot2 %obj.xfoot2==0
                theta1 = u;
                obj.xfoot1 = pf_x(1)+((pf_x(2)*cos(theta1))/sin(theta1)); % x position of r1
                obj.yfoot1 = 0; % y position of r1
            elseif obj.xfoot1 < obj.xfoot2 %obj.xfoot1==0
                theta2 = u;
                obj.xfoot2 = pf_x(1)+((pf_x(2)*cos(theta2))/sin(theta2)); % x position of r2
                obj.yfoot2 = 0; % y position of r2
            end
        end
        
        function [pf_x,mode,status]=r2stance2flight(~,mode,~,ps2_x,~) %(obj,mode,t,x,u)
            pf_x=[ps2_x(1:2);ps2_x(5:6)];
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=3;
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
        
        function [ps1_x,mode,status]=flight2r1stance(obj,mode,~,pf_x,u) %(obj,mode,t,x,u)
            ps1_x=pf_x(1:4);
            if(mode~=3)
                error('Incorrect mode');
            else
                mode=1;
            end
            status=0;
            if obj.xfoot1 > obj.xfoot2 %obj.xfoot2==0
                theta1 = u;
                obj.xfoot1 = pf_x(1)+((pf_x(2)*cos(theta1))/sin(theta1)); % x position of r1
                obj.yfoot1 = 0; % y position of r1
            elseif obj.xfoot1 < obj.xfoot2 %obj.xfoot1==0
                theta2 = u;
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
            vis.display_dt = .001;
            vis.playback(ytraj,struct('slider','true'));
        end
    end
end