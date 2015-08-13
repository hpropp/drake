classdef BipedalSLIPVisualizer < Visualizer
    % the visualization of the bipedalSLIP model
    
    properties
        xfoot1; % x position of r1
        xfoot2; % x position of r2
        yfoot1; % y position of r1
        yfoot2; % y position of r2
    end
    
    methods
        function obj = BipedalSLIPVisualizer(slip)
            typecheck(slip,'BipedalSLIP');
            obj = obj@Visualizer(getOutputFrame(slip));
            
            obj.xfoot1 = slip.xfoot1; % x position of r1
            obj.xfoot2 = slip.xfoot2; % x position of r2
            obj.yfoot1 = slip.yfoot1; % y position of r1
            obj.yfoot2 = slip.yfoot2; % y position of r2
        end
        
        %function obj = BipedalSLIPVisualizer()
        %   obj = obj@Visualizer([]);
        %end % a debugging visualizer function
        
        function draw(obj,~,x) %(obj,t,x) where x is a 1x4 matrix
            hip = x(1:2); % position of the hip
            
            clf
            
            % needed for the spring drawings
            % lx=linspace(0,1,51);
            % ly=.075*sin(8*pi*lx);
            % lx = [0,.3,.5*lx+.3,.8,1];
            % ly = [0,0,ly,0,0];
            
            % the first spring leg, drawn with the spring
            % R = rotmat(theta+pi);
            % lpts = repmat(hip,1,numel(lx))+R*[r1*lx;ly];
            % plot(lpts(1,:),lpts(2,:),'Color',[0 0 0],'LineWidth',2);
            
            % the first spring leg, drawn without the spring
            line([obj.xfoot1, hip(1)], [obj.yfoot1, hip(2)]);
            
            hold on
            
            % the second spring leg, drawn without the spring
            line([obj.xfoot2, hip(1)], [obj.yfoot2, hip(2)]);
            
            % the second spring leg, drawn with the spring
            % R2 = rotmat(theta2+pi);
            % lpts = repmat(hip,1,numel(lx))+R2*[(r2)*lx;ly];
            % plot(lpts(1,:),lpts(2,:),'Color',[0 0 0],'LineWidth',2);
            
            
            t = 0:0.1:2*pi;
            line(hip(1)+0.15*sin(t),hip(2)+0.15*cos(t),'Color',[0 0 0]);
            fill(hip(1)+0.15*sin(t),hip(2)+0.15*cos(t),MITlightgray); %[ 0.502 1.000 1.000 ]);
            
            axis equal;
            
            if (isempty(obj.axis))
                v(1) = 4*floor(hip(1)/4)-.2;
                v(2) = v(1)+4.4;
                v(3) = -0.5; %-.1;
                v(4) = 3;
                line([v(1)-1 v(2)+1],[0 0],'LineWidth',1.5,'Color',MITred);
                axis(v)
            else
                line(obj.axis(1:2),[0 0],'LineWidth',1.5,'Color',MITred);
                axis(obj.axis);
            end
            
            drawnow;
        end
    end
end
