classdef Environment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here

    properties
    end

    methods (Static)
        function [p1,s1,b1,b2,b3,b4,b5,b6,L1,L2,g1,e1,f1,w1] = simEnvironment(tablePOS,ShakerPOS,Bottle1POS,Bottle2POS,Bottle3POS,Button1POS,Button2POS,Button3POS,LightCurt1POS,LightCurt2POS,GlassPOS,EStopPOS,FextPOS);
            %workspace = [-5 5 -5 5 -0.02 4.5];
            workspace =[-2.5 2.5 -2 2 -0.02 5];
            axis(workspace);
            view([43.5,17.35]);
            camzoom(1);
            cla;
            hold on;
            scale =0.1;
            BottleScale = 2.5;
            ButtonScale = 0.001;
            LightScale = 0.005;
            GlassScale = 0.003;

            % Place Floor
            f1 = imread('Woodflr.jpeg');
            % xImage = [self.workspace(1) self.workspace(2); self.workspace(1) self.workspace(2)];   % x data (constant)
            % yImage = [self.workspace(3) self.workspace(3); self.workspace(4) self.workspace(4)];   % y data (min to max of the workspace)
            xImage = [workspace(1) workspace(2); workspace(1) workspace(2)];   % x data (constant)
            yImage = [workspace(3) workspace(3); workspace(4) workspace(4)];   % y data (min to max of the workspace)
            zImage = [0 0; 0 0];           % z data (min to max of the workspace)

            % Create the surface and map the image onto it
            surf(xImage, yImage, zImage, 'CData', f1, 'FaceColor', 'texturemap');
            %Place table
            p1= PlaceObject('tableBrown2.1x1.4x0.5m.ply',tablePOS);

            %Place shaker
            s1= PlaceObject('ShakerBody.ply',ShakerPOS);
            s1verts = get(s1, 'Vertices');
            s1Pos = ShakerPOS;
            s1centred = s1verts - s1Pos;
            s1RM = trotx(0,"deg");  % Rotation by 90 degrees around Y-axis
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
            % b1Scaled = b1Rotated * self.BottleScale;  % Scale the rotated vertices
            b1Scaled = b1Rotated * BottleScale;  % Scale the rotated vertices
            b1Trans = b1Scaled + b1Pos;
            set(b1, 'Vertices', b1Trans);

            b2 = PlaceObject('KrakenBottle.ply', Bottle2POS);
            b2verts = get(b2, 'Vertices');
            b2Pos = Bottle2POS;
            b2centred = b2verts - b2Pos;
            b2RM = trotx(-pi/2);  % Rotation by 90 degrees around Y-axis
            b2Rotated = (b2RM(1:3, 1:3) * b2centred')';
            b2Scaled = b2Rotated * BottleScale;  % Scale the rotated vertices
            b2Trans = b2Scaled + b2Pos;
            set(b2, 'Vertices', b2Trans);

            b3 = PlaceObject('KrakenBottle.ply', Bottle3POS);
            b3verts = get(b3, 'Vertices');
            b3Pos = Bottle3POS;
            b3centred = b3verts - b3Pos;
            b3RM = trotx(-pi/2);  % Rotation by 90 degrees around Y-axis
            b3Rotated = (b3RM(1:3, 1:3) * b3centred')';
            b3Scaled = b3Rotated * BottleScale;  % Scale the rotated vertices
            b3Trans = b3Scaled + b3Pos;
            set(b3, 'Vertices', b3Trans);

            %place buttons
            b4 = PlaceObject('Button.ply',Button1POS);
            b4verts = get(b4, 'Vertices');
            b4Pos = Button1POS;
            b4centred = b4verts - b4Pos;
            b4Scaled = b4centred * ButtonScale;  % Scale the rotated vertices
            b4Trans = b4Scaled + b4Pos;
            set(b4, 'Vertices', b4Trans);

            b5 = PlaceObject('Button.ply', Button2POS);
            b5verts = get(b5, 'Vertices');
            b5Pos = Button2POS;
            b5centred = b5verts - b5Pos;
            b5Scaled = b5centred * ButtonScale;  % Scale the rotated vertices
            b5Trans = b5Scaled + b5Pos;
            set(b5, 'Vertices', b5Trans);

            b6 = PlaceObject('Button.ply', Button3POS);
            b6verts = get(b6, 'Vertices');
            b6Pos = Button3POS;
            b6centred = b6verts - b6Pos;
            b6Scaled = b6centred * ButtonScale;  % Scale the rotated vertices
            b6Trans = b6Scaled + b6Pos;
            set(b6, 'Vertices', b6Trans);

            %Place Light Curtains
            L1 = PlaceObject('LightCurt.ply', LightCurt1POS);
            L1verts = get(L1, 'Vertices');
            L1Pos = LightCurt1POS;
            L1centred = L1verts - L1Pos;
            L1RM = trotz(pi/2);
            stretchFactor1 = 3.7 / 0.75;
            scalingMatrix1 = diag([LightScale,stretchFactor1 * LightScale, LightScale]);
            L1Trans = (L1RM(1:3, 1:3) * (scalingMatrix1 * L1centred') )';
            L1Trans = L1Trans + L1Pos;
            set(L1, 'Vertices', L1Trans);

            L2 = PlaceObject('LightCurt.ply', LightCurt2POS);
            L2verts = get(L2, 'Vertices');
            L2Pos = LightCurt2POS;
            L2centred = L2verts - L2Pos;
            L2RM = eye(3);
            stretchFactor2 = 1.1 / 0.75;
            scalingMatrix2 = diag([LightScale,stretchFactor2 * LightScale, LightScale]);
            L2Trans = (L2RM(1:3, 1:3) * (scalingMatrix2 * L2centred') )';
            L2Trans = L2Trans + L2Pos;
            set(L2, 'Vertices', L2Trans);

            %Place Glass
            g1 = PlaceObject('Glass.ply', GlassPOS);
            g1verts = get(g1, 'Vertices');
            g1Pos = GlassPOS;
            g1centred = g1verts - g1Pos;
            g1RM = trotx(pi/2);
            g1centred_scaled = GlassScale * g1centred;  % Still n x 3
            g1centred_scaled = [g1centred_scaled, ones(size(g1centred_scaled, 1), 1)];  % Convert to n x 4
            g1Trans = (g1centred_scaled * g1RM')';
            g1Trans = g1Trans(1:3, :)' + g1Pos;
            set(g1, 'Vertices', g1Trans);

            %place Estop
            e1 = PlaceObject('emergencyStopButton.ply', EStopPOS);

            %place Fireextinguisher
            f1 = PlaceObject('fireExtinguisher.ply', FextPOS);


        end
    end
end
