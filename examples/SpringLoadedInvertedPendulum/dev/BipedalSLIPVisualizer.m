classdef BipedalSLIPVisualizer < Visualizer
    % the visualization of the bipedalSLIP model
    
    methods
        function obj = BipedalSLIPVisualizer(slip)
            typecheck(slip,'BipedalSLIP');
            obj = obj@Visualizer(getOutputFrame(slip));
        end
        
        %function obj = BipedalSLIPVisualizer()
        %   obj = obj@Visualizer([]);
        %end % a debugging visualizer function
        
        function draw(obj,~,x) %(obj,t,x) where x is a 1x4 matrix
            hip = x(1:2); % position of the hip
            
            yfoot1 = 0;
            xfoot1 = 0.25;
            r1 = sqrt((x(1)-xfoot1)^2+x(2)^2);
            r2 = 1;
            
            theta1 = atan2(x(2),xfoot1-x(1));
            theta2 = pi/3;
            
            % xfoot1 = x(1)+((x(2)*cos(theta1))/sin(theta1));
            yfoot2 = x(2)-(r2*sin(theta2));
            xfoot2 = x(1)+(r2*cos(theta2));
            
            clf
            
            % lx=linspace(0,1,51);
            % ly=.075*sin(8*pi*lx);
            % lx = [0,.3,.5*lx+.3,.8,1];
            % ly = [0,0,ly,0,0];
            
            % this controls the first spring leg
            % R = rotmat(theta+pi);
            % lpts = repmat(hip,1,numel(lx))+R*[r1*lx;ly];
            % plot(lpts(1,:),lpts(2,:),'Color',[0 0 0],'LineWidth',2);
            
            % the first spring leg, drawn w/o the spring
            line([xfoot1, hip(1)], [yfoot1, hip(2)]);
            
            hold on
            
            % the second spring leg, drawn w/o the spring
            line([xfoot2, hip(1)], [yfoot2, hip(2)]);
            
            % this controls the second spring leg
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
