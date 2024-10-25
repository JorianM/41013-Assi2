classdef RG2Finger < RobotBaseClass
    %% LinearUR5 UR5 on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'RG2Finger';
    end
    
    methods
%% Define robot Function 
        function self = RG2Finger(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;% * trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot();         
        end
        function CreateModel(self)   
            link(1) = Link('d',0.1,'a',-0.015,'alpha',pi/2,'qlim',deg2rad([0 0]),'offset',pi/2);  
            link(2) = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-15 15]),'offset',-pi/2);
            self.model = SerialLink(link,'name',self.name);
            % self.model.plotopt = {'noshadow','noarrow','noshading','nowrist','nojaxes'};
        end
    end
end
