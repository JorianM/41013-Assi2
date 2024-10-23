classdef Environment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Property1
        workspace = [-2.5 2.5 -2 2 -0.02 5];
        scale =0.1;
        BottleScale = 2.5;
        ButtonScale = 0.001;

    end
    
    methods 
        function [p1,s1,b1,b2,b3,b4,b5,b6] = simEnvironment(self,tablePOS,ShakerPOS,Bottle1POS,Bottle2POS,Bottle3POS,Button1POS,Button2POS,Button3POS)
        
     
        hold on;
        axis(self.workspace);
        
% Place Floor 
        f1 = imread('Woodflr.jpeg');
        xImage = [self.workspace(1) self.workspace(2); self.workspace(1) self.workspace(2)];   % x data (constant)
        yImage = [self.workspace(3) self.workspace(3); self.workspace(4) self.workspace(4)];   % y data (min to max of the workspace)
        zImage = [0 0; 0 0];           % z data (min to max of the workspace)

        % Create the surface and map the image onto it
        surf(xImage, yImage, zImage, 'CData', f1, 'FaceColor', 'texturemap');
%Place table
        p1= PlaceObject('tableBrown2.1x1.4x0.5m.ply',tablePOS);

%Place shaker
        s1= PlaceObject('Shaker.ply',ShakerPOS);
        s1verts = get(s1, 'Vertices');
        s1Pos = ShakerPOS;
        s1centred = s1verts - s1Pos;
        s1RM = trotx(pi/2);  % Rotation by 90 degrees around Y-axis
        s1Trans = (s1RM(1:3, 1:3) * s1centred')';
        s1Trans = s1Trans + s1Pos;
        set(s1, 'Vertices', s1Trans);


% Place Bar1
        [f,v,data] = plyread('Bar.ply','tri'); % Load the bar object with vertex data

        % Check if the file contains color information
        % if isfield(data, 'vertex') && isfield(data.vertex, 'red')
        %     % Scale the colors to be 0-to-1 (they are originally 0-to-255)
        %     vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        %     disp("has colour")
        % else
            % If no color info, use a default color (e.g., grey)
            vertexColors = repmat([0.396, 0.263, 0.129], size(v, 1), 1); % Grey color
        %     disp("No colour")
        % end

        scaling_factor = 0.025;
        v = v * scaling_factor;
        O1RM = trotx(-pi/2); % Create rotation matrix
        vRotated = (O1RM(1:3, 1:3) * v')'; % Rotate the vertices
        O1Pos = [2.25, -1.2, 0]; % Desired position
        vTranslated = vRotated + O1Pos; % Translate the vertices\
        trisurf(f, vTranslated(:,1), vTranslated(:,2), vTranslated(:,3), ...
            'FaceVertexCData', vertexColors, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

        % Adjust lighting for the scene
        camlight('headlight');
        lighting gouraud;


%place bottles
        b1 = PlaceObject('KrakenBottle.ply', Bottle1POS);
        b1verts = get(b1, 'Vertices');
        b1Pos = Bottle1POS;
        b1centred = b1verts - b1Pos;
        b1RM = trotx(-pi/2);  % Rotation by 90 degrees around Y-axis
        b1Rotated = (b1RM(1:3, 1:3) * b1centred')';
        b1Scaled = b1Rotated * self.BottleScale;  % Scale the rotated vertices
        b1Trans = b1Scaled + b1Pos;
        set(b1, 'Vertices', b1Trans);

        b2 = PlaceObject('KrakenBottle.ply', Bottle2POS);
        b2verts = get(b2, 'Vertices');
        b2Pos = Bottle2POS;
        b2centred = b2verts - b2Pos;
        b2RM = trotx(-pi/2);  % Rotation by 90 degrees around Y-axis
        b2Rotated = (b2RM(1:3, 1:3) * b2centred')';
        b2Scaled = b2Rotated * self.BottleScale;  % Scale the rotated vertices
        b2Trans = b2Scaled + b2Pos;
        set(b2, 'Vertices', b2Trans);

        b3 = PlaceObject('KrakenBottle.ply', Bottle3POS);
        b3verts = get(b3, 'Vertices');
        b3Pos = Bottle3POS;
        b3centred = b3verts - b3Pos;
        b3RM = trotx(-pi/2);  % Rotation by 90 degrees around Y-axis
        b3Rotated = (b3RM(1:3, 1:3) * b3centred')';
        b3Scaled = b3Rotated * self.BottleScale;  % Scale the rotated vertices
        b3Trans = b3Scaled + b3Pos;
        set(b3, 'Vertices', b3Trans);

%place buttons
        b4 = PlaceObject('Button.ply',Button1POS);
        b4verts = get(b4, 'Vertices');
        b4Pos = Button1POS;
        b4centred = b4verts - b4Pos;
        b4Scaled = b4centred * self.ButtonScale;  % Scale the rotated vertices
        b4Trans = b4Scaled + b4Pos;
        set(b4, 'Vertices', b4Trans);

        b5 = PlaceObject('Button.ply', Button2POS);
        b5verts = get(b5, 'Vertices');
        b5Pos = Button2POS;
        b5centred = b5verts - b5Pos;
        b5Scaled = b5centred * self.ButtonScale;  % Scale the rotated vertices
        b5Trans = b5Scaled + b5Pos;
        set(b5, 'Vertices', b5Trans);

        b6 = PlaceObject('Button.ply', Button3POS);
        b6verts = get(b6, 'Vertices');
        b6Pos = Button3POS;
        b6centred = b6verts - b6Pos;
        b6Scaled = b6centred * self.ButtonScale;  % Scale the rotated vertices
        b6Trans = b6Scaled + b6Pos;
        set(b6, 'Vertices', b6Trans);
        

        end

%  function obj = Environment(inputArg1,inputArg2) % maybe do function [ingredient1 ingredient2 ingredient3, ...]= simulateEnvironment(ingredient1Pos, ingredient2Pos, ingredient3Pos, ...)
%             %ENVIRONMENT Construct an instance of this class
%             %   Detailed explanation goes here
%             obj.Property1 = inputArg1 + inputArg2;
%             %
%             % draft functionality
%             cla;
%             hold on;
% 
%             % Ingredients
%             % ingredients getting placed
%             % ingredient1 = PlaceObject('vodka?.ply' ,ingredient1Pos);
%             % .
%             % .
%             % .
% 
%             % Bar
%             % Obj1 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0.8,0]);
% 
%             % Safety
%             % fireExtinguisherObj = PlaceObject('fireExtinguisher.ply',[2.2,2.2,0]);
%             % eStopObj = PlaceObject('emergencyStopButton.ply', [0, 2.5, 1.2]);
%             % barrierObj1 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0.8,0]);
% 
%         end
% 
%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
% 
% 
% disp("aaaaaAAAAAaaaarrrrg")
% 


end
end

