classdef Control < handle
    properties
    end
    methods (Static)
        function finalShakerPosition = PlotShaker(rbt, qTj)
            % Read the shaker mesh data in
            [f, v, ~] = plyread('Shaker.ply', 'tri');

            shakerMesh_h = []; % Initialize the handle for the shaker mesh
            finalShakerPosition = [];


            for i = 1:size(qTj, 1)
                q = qTj(i, :);
                rbt.model.animate(q);

                % Get the current shaker position using forward kinematics
                currentShakerPose = rbt.model.fkine(q);

                % Adjust shaker position 
                shakerPosition = transl(currentShakerPose);  % Extract translation part (x, y, z)
                
                % Store the last Shaker position
                finalShakerPosition = shakerPosition;  

                % Delete the previous shaker plot, if it exists
                if ishandle(shakerMesh_h)
                    delete(shakerMesh_h);  % This ensures the mesh is properly deleted
                end

                % Re-plot the shaker at the new position
                UpdatedPoints = [v(:,1) + shakerPosition(1), v(:,2) + shakerPosition(2), v(:,3) + shakerPosition(3)];
                shakerMesh_h = trisurf(f, UpdatedPoints(:, 1), UpdatedPoints(:, 2), UpdatedPoints(:, 3), ...
                    'FaceColor', 'k', 'EdgeColor', 'none', 'EdgeLighting', 'none');

                % Pause for smooth visualization
                pause(0.05);
            end
            
            % Delete the shaker mesh at the end
            if ishandle(shakerMesh_h)
                delete(shakerMesh_h);
            end
            
        end
    
        function qT = CreateTrajectory(rbt,objPos) %,armManipulate)
            steps = 100;
            q = rbt.model.getpos();
            T = transl(objPos)*trotx(pi)*troty(0)*trotz(0);
            q2 = wrapToPi(rbt.model.ikcon(T));%,armManipulate));
            qT = jtraj(q,q2,steps);

        end
    
        function  moveToPos(rbt,qTj)

            for i = 1:size(qTj,1)
                q = qTj(i,:);
                rbt.model.animate(q);
                pause(0.05)
            end

        end
    
    
    end
end

