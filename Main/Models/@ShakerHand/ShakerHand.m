classdef ShakerHand < RobotBaseClass
    %% LinearUR5 UR5 on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'ShakerHand';
    end
    
    methods
%% Define robot Function 
        function self = ShakerHand(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;% * trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot();         
        end
        function CreateModel(self)   
            link(1) = Link('d',0,'a',-0,'alpha',0,'qlim',deg2rad([0 0]),'offset',0);  
            % link(1) = Link('d',0.2,'a',0.2,'alpha',0,'qlim',deg2rad([-15 15]),'offset',0); 
            link(2) = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-15 15]),'offset',0);
            self.model = SerialLink(link,'name',self.name);
            % self.model.plotopt = {'noshadow','noarrow','noshading','nowrist','nojaxes'};
        end
    end
end
