classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Property1
        workspace = [-2.5 2.5 -2.5 2.5 0 2.5];
        axis = (workspace);
    end
    
    methods (Static)
        function obj = Environment(inputArg1,inputArg2) % maybe do function [ingredient1 ingredient2 ingredient3, ...]= simulateEnvironment(ingredient1Pos, ingredient2Pos, ingredient3Pos, ...)
            %ENVIRONMENT Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
            %
            % draft functionality
            cla;
            hold on;
            
            % Ingredients
            % ingredients getting placed
            % ingredient1 = PlaceObject('vodka?.ply' ,ingredient1Pos);
            % .
            % .
            % .

            % Bar
            % Obj1 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0.8,0]);

            % Safety
            % fireExtinguisherObj = PlaceObject('fireExtinguisher.ply',[2.2,2.2,0]);
            % eStopObj = PlaceObject('emergencyStopButton.ply', [0, 2.5, 1.2]);
            % barrierObj1 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0.8,0]);

        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

disp("aaaaaAAAAAaaaarrrrg")