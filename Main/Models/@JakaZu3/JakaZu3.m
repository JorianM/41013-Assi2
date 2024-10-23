classdef JakaZu3 < RobotBaseClass
    %% LinearUR5 UR5 on a non-standard linear rail created by a student

    properties(Access = public)
        plyFileNameStem = 'JakaZu3';
    end

    methods
        %% Define robot Function
        function self = JakaZu3(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;% * trotx(pi/2); %* troty(pi/2);

            self.PlotAndColourRobot();
        end

        % %% Constructor
        %         function self = JakaZu3(baseTr,useTool,toolFilename)
        %             if nargin < 3
        %                 if nargin == 2
        %                     error('If you set useTool you must pass in the toolFilename as well');
        %                 elseif nargin == 0 % Nothing passed
        %                     baseTr = transl(0,0,0);
        %                 end
        %             else % All passed in
        %                 self.useTool = useTool;
        %                 toolTrData = load([toolFilename,'.mat']);
        %                 self.toolTr = toolTrData.tool;
        %                 self.toolFilename = [toolFilename,'.ply'];
        %             end
        %
        %             self.CreateModel();
        % 			self.model.base = self.model.base.T * baseTr;
        %             self.model.tool = self.toolTr;
        %             self.PlotAndColourRobot();
        %             drawnow
        %         end

        %% Create the robot model
        % function CreateModel(self)
        %     % Create the JakaZu from https://iopscience.iop.org/article/10.1088/1361-6501/ac9920
        %     % from https://www.jakarobotics.com/products/jaka-zu/zu-3/#
        %     link(1) = Link('d',0.1506,'a',-0.4,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
        %     link(2) = Link('d',-0.1150,'a',-0,'alpha',0,'qlim', deg2rad([-85 265]), 'offset',0);%-360 360
        %     link(3) = Link('d',0.1095,'a',0.2460,'alpha',0,'qlim', deg2rad([-175 175]), 'offset', 0);
        %     link(4) = Link('d',-0.1175,'a',0.2280,'alpha',pi/2,'qlim',deg2rad([-85 265]),'offset', 0);
        %     link(5) = Link('d',-0.1175,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
        %     link(6) = Link('d',-0.1050,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
        %
        %     % linearur3e
        %     % link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
        %     % link(2) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-90 90]), 'offset',0);%-360 360
        %     % link(3) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-130 130]), 'offset', 0);
        %     % link(4) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
        %     % link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
        %     % link(6) = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
        %     % Incorporate joint limits
        %     % link(1).qlim = [-0.8 -0.01];
        %     link(1).qlim = [-180 180]*pi/180;
        %     link(2).qlim = [-180 180]*pi/180;
        %     link(3).qlim = [-180 180]*pi/180;
        %     link(4).qlim = [-220 220]*pi/180;
        %     link(5).qlim = [-220 220]*pi/180;
        %     link(6).qlim = [-220 220]*pi/180;
        %
        %     %
        %     % link(4).offset = -pi/2;
        %     % link(6).offset = -pi/2;
        %
        %
        %     self.model = SerialLink(link,'name',self.name);
        %
        %
        % end

        % Create the robot model for Zu3
        function CreateModel(self)
            % Link parameters for the Zu3 model
            % Create the JakaZu from https://iopscience.iop.org/article/10.1088/1361-6501/ac9920
            % from https://www.jakarobotics.com/products/jaka-zu/zu-3/#
            link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',-pi/2);
            link(2) = Link('d',0,'a',0.2455,'alpha',0,'qlim', deg2rad([-90 90]), 'offset',deg2rad(90));%-360 360
            link(3) = Link('d',0,'a',0.2280,'alpha',0,'qlim', deg2rad([-130 130]), 'offset', 0);
            link(4) = Link('d', 0.118, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', deg2rad([-85 265]));%0.1175,
            link(5) = Link('d', 0.1175,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',	0.106,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            % Create the SerialLink model
            self.model = SerialLink(link, 'name', self.name);
        end



    end
end
