classdef BipedalSLIP < HybridDrakeSystem
    % a script combining all of the other bipedalSLIP classes
    
    properties
        r_rest = 1; % rest length of leg springs (m)
        m_hip = 1; % mass (kg)
        k = 100; % stiffness spring coefficient (aka elastic restoring force)
        g = 9.81; % gravity (m/s^2)
        alpha0_r2 = pi/3;
        alpha0_r1 = (5*pi)/7;
    end
    
    methods
        function obj = BipedalSLIP()
            obj = obj@HybridDrakeSystem(...
                1, ... % number of inputs
                12); % number of outputs
            
            obj = setInputFrame(obj,CoordinateFrame('SLIPInput',1,'u',{'alpha0_r2'})); % alpha0_r2 is the angle of attack
            obj = setOutputFrame(obj,CoordinateFrame('SLIPOutput',12,'y',{'x','y','r1','theta1','r2','theta2','xdot','ydot','r1dot','theta1dot','r2dot','theta2dot'}));
            
            % singleSupport mode (one leg on the ground, one leg in the air)
            pSingleSupport=BipedalSLIPSingleSupport(obj);
            [obj, ps1_mode]=obj.addMode(pSingleSupport); % mode 1
            pSingleSupport=BipedalSLIPSingleSupport(obj);
            [obj, ps2_mode]=obj.addMode(pSingleSupport); % mode 2
            
            % doubleSupport mode, where both legs are on the ground
            pDoubleSupport=BipedalSLIPDoubleSupport(obj);
            [obj, pd1_mode]=obj.addMode(pDoubleSupport); % mode 3
            pDoubleSupport=BipedalSLIPDoubleSupport(obj);
            [obj, pd2_mode]=obj.addMode(pDoubleSupport); % mode 4
            
            % flight mode (both legs in the air)
            pFlight=BipedalSLIPFlight(obj);
            [obj, pf1_mode]=obj.addMode(pFlight); % mode 5
            pFlight=BipedalSLIPFlight(obj);
            [obj, pf2_mode]=obj.addMode(pFlight); % mode 6
            
            % walking transitions
            obj=addTransition(obj,ps1_mode,@penetratewalkGuard,@single1todouble1,true,true,pd1_mode);
            obj=addTransition(obj,pd1_mode,andGuards(obj,@restlengthwalkGuard,@legextensionwalkGuard),@double1tosingle2,true,true,ps2_mode);
            obj=addTransition(obj,ps2_mode,@penetratewalkGuard,@single2todouble2,true,true,pd2_mode);
            obj=addTransition(obj,pd2_mode,andGuards(obj,@restlengthwalkGuard,@legextensionwalkGuard),@double2tosingle1,true,true,ps1_mode);
            % running transitions
            obj=addTransition(obj,ps1_mode,andGuards(obj,@restlengthrunGuard,@legextensionrunGuard),@stance1toflight1,true,true,pf1_mode);
            obj=addTransition(obj,pf1_mode,@penetratewalkGuard,@flight1tostance2,true,true,ps2_mode);
            obj=addTransition(obj,ps2_mode,andGuards(obj,@restlengthrunGuard,@legextensionrunGuard),@stance2toflight2,true,true,pf2_mode);
            obj=addTransition(obj,pf2_mode,@penetratewalkGuard,@flight2tostance1,true,true,ps1_mode);
        end
        
        function ps_walkguard = penetratewalkGuard(obj,t,ps_x,u) %(obj,t,x,u)
            ps_walkguard = ps_x(2)-(obj.r_rest*sin(u));
        end
        
        function pd_restlguard = restlengthwalkGuard(obj,t,pd_x,u) %(obj,t,x,u)
            r1 = sqrt((pd_x(1)-pd_x(3))^2+pd_x(2)^2);
            pd_restlguard = -(r1-obj.r_rest);
        end
        
        function pd_legextendguard = legextensionwalkGuard(obj,t,pd_x,u) %(obj,t,x,u)
            r1dot = (((pd_x(1)-pd_x(3))*pd_x(7))+(pd_x(2)*pd_x(8)))/sqrt((pd_x(1)^2)-(2*pd_x(3)*pd_x(1))+(pd_x(2)^2)+(pd_x(3)^2));
            pd_legextendguard = -r1dot;
        end
        
        %         function pf_runguard = penetraterunGuard(obj,t,pf_x,u) %(obj,t,x,u)
        %             pf_runguard = pf_x(2)-(obj.r_rest*sin(u));
        %         end
        
        function ps_restlguard = restlengthrunGuard(obj,t,ps_x,u) %(obj,t,x,u)
            r1 = sqrt((ps_x(1)-ps_x(3))^2+ps_x(2)^2);
            ps_restlguard = -(r1-obj.r_rest);
        end
        
        function pf_legextendguard = legextensionrunGuard(obj,t,ps_x,u) %(obj,t,x,u)
            r1dot = (((ps_x(1)-ps_x(3))*ps_x(5))+(ps_x(2)*ps_x(6)))/sqrt((ps_x(1)^2)-(2*ps_x(3)*ps_x(1))+(ps_x(2)^2)+(ps_x(3)^2));
            pf_legextendguard = -r1dot;
        end
        
        function [pd_x,mode,status]=single1todouble1(obj,mode,t,ps_x,u) %(obj,mode,t,x,u)
            pd_x=[ps_x(1:4);ps_x(1)+(obj.r_rest*cos(u));0;ps_x(5:6)];
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=3;
            end
            status=(pd_x(7)<0);  % terminate if xdot < 0
        end
        
        function [ps_x,mode,status]=double1tosingle2(obj,mode,t,pd_x,u) %(obj,mode,t,x,u)
            ps_x=[pd_x(1:4);pd_x(7:8)];
            if(mode~=3)
                error('Incorrect mode');
            else
                mode=2;
            end
            status=0;
        end
        
        function [pd_x,mode,status]=single2todouble2(obj,mode,t,ps_x,u) %(obj,mode,t,x,u)
            pd_x=[ps_x(1:4);ps_x(1)+(obj.r_rest*cos(u));0;ps_x(5:6)];
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=4;
            end
            status=(pd_x(7)<0);  % terminate if xdot < 0
        end
        
        function [ps_x,mode,status]=double2tosingle1(obj,mode,t,pd_x,u) %(obj,mode,t,x,u)
            ps_x=[pd_x(1:4);pd_x(7:8)];
            if(mode~=4)
                error('Incorrect mode');
            else
                mode=1;
            end
            status=0;
        end
        
        function [pf_x,mode,status]=stance1toflight1(obj,mode,t,ps_x,u) %(obj,mode,t,x,u)
            pf_x=[ps_x(1:2);ps_x(5:6)];
            if(mode~=1)
                error('Incorrect mode');
            else
                mode=5;
            end
            status=(pf_x(3)<0);  % terminate if xdot < 0
        end
        
        function [ps_x,mode,status]=flight1tostance2(obj,mode,t,pf_x,u) %(obj,mode,t,x,u)
            ps_x=[pf_x(1:2);pf_x(1)+((pf_x(2)*cos(obj.alpha0_r2))/sin(obj.alpha0_r2));0;pf_x(3:4)];
            if(mode~=5)
                error('Incorrect mode');
            else
                mode=2;
            end
            status=0;
        end
        
        function [pf_x,mode,status]=stance2toflight2(obj,mode,t,ps_x,u) %(obj,mode,t,x,u)
            pf_x=[ps_x(1:2);ps_x(5:6)];
            if(mode~=2)
                error('Incorrect mode');
            else
                mode=6;
            end
            status=(pf_x(3)<0);  % terminate if xdot < 0
        end
        
        function [ps_x,mode,status]=flight2tostance1(obj,mode,t,pf_x,u) %(obj,mode,t,x,u)
            ps_x=[pf_x(1:2);pf_x(1)+((pf_x(2)*cos(obj.alpha0_r2))/sin(obj.alpha0_r2));0;pf_x(3:4)];
            if(mode~=6)
                error('Incorrect mode');
            else
                mode=1;
            end
            status=0;
        end
        
        function theta=angleOfAttackControl(obj,yd,y0,xdot0)
            aoaControl=@(theta) obj.apex2apex(theta,y0,xdot0)-yd;
            theta=fzero(aoaControl,pi/8);
        end
    end
    
    methods (Static)
        function run()
            r = BipedalSLIP();
            vis = BipedalSLIPVisualizer(r);
            
            aoa_command = setOutputFrame(ConstantTrajectory(pi/3),getInputFrame(r));
            ytraj = simulate(cascade(aoa_command,r),[0 .83]);
            vis.display_dt = .001;
            vis.playback(ytraj,struct('slider','true'));
        end
    end
end