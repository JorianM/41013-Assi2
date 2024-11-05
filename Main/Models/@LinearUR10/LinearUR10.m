classdef LinearUR10 < RobotBaseClass
    %% LinearUR10 UR10 on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'LinearUR10';
    end
    
    methods
%% Constructor
        function self = LinearUR10(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)   
            % Create the UR10 model mounted on a linear rail
            link(1) = Link([pi    0           0        pi/2    1]); % PRISMATIC Link
            % link(2) = Link([0     0.1697      0        -pi/2   0]);
            link(2) = Link('d',0.1697 ,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',pi/2);
            link(3) = Link([0     0.176       0.6129   -pi     0]);
            link(4) = Link([0     0.12781     0.5716	pi      0]);
            link(5) = Link([0     0.1157      0        -pi/2	0]);
            % link(6) = Link([0     0.1157     0        pi/2	0]);
            link(6) = Link('d',0.1157,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',pi);
            % link(7) = Link([0     0.06037           0        0       0]);
            % link(2) = Link('d',0.128,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            % link(3) = Link('d',0,'a',-0.6127,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            % link(4) = Link('d',0,'a',-0.5716,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            % link(5) = Link('d',0.16389,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            % link(6) = Link('d',0.1157,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            % link(7) = Link('d',0.09037,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            % Incorporate joint limits
            link(1).qlim = [-0.8   -0.01]; % Must be negative
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-45 45]*pi/180;
            link(4).qlim = [-360 360]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            link(7).qlim = [-360 360]*pi/180;
        
            link(3).offset = -pi/2;
            % link(5).offset = -pi/2;
        
            self.model = SerialLink(link,'name',self.name);
        end     
    end
end