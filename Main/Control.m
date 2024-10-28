classdef Control < handle
    properties
    end
    methods (Static)
    


        
        function finalShakerPosition = PlotShaker(rbt, qTj,finger,mfinger)
            % Read the shaker mesh data in
            [f, v, ~] = plyread('ShakerBody.ply', 'tri');

            shakerMesh_h = []; % Initialize the handle for the shaker mesh
            finalShakerPosition = [];
            q_f1_end = deg2rad([25, 0]);
            q_f2_end = deg2rad([25, 0]);

            % Get the initial positions for both fingers
            q_f1_start = finger.model.getpos();
            q_f2_start = mfinger.model.getpos();

            % Generate the joint trajectories for each finger
            q_f1_traj = jtraj(q_f1_start, q_f1_end, size(qTj, 1));
            q_f2_traj = jtraj(q_f2_start, q_f2_end, size(qTj, 1));

            for i = 1:size(qTj, 1)
                % if app.Running
                % if ~app.Running
                %     break; % Exit loop if app is not running
                % end
                q = qTj(i, :);
                rbt.model.animate(q);
                base = rbt.model.fkineUTS(q);
                % Set the transformed base for each finger and animate
                finger.model.base = base;% * trotx(pi/2);
                finger.model.animate(q_f1_traj(i, :));

                mfinger.model.base = base * trotz(pi);% * troty(pi) * trotx(-pi/2);
                mfinger.model.animate(q_f2_traj(i, :));

                % Pause to create a smooth animation
                pause(0.02);
                %     for i = 1:size(qTj,1)
                %         q = qTj(i,:);
                %         rbt.model.animate(q);
                %
                %         pause(0.05)
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
                % end
            end

            % Delete the shaker mesh at the end
            if ishandle(shakerMesh_h)
                delete(shakerMesh_h);
            end

        end

        function qT = CreateTrajectory(rbt,objPos,steps) %,armManipulate)
            % %steps = 200;%100
            % q = rbt.model.getpos();
            % T = transl(objPos)*trotx(pi)*troty(0)*trotz(0)
            % % T = transl(objPos+[0,0,0.2])*trotx(pi)*troty(0)*trotz(0);
            % q2 = wrapToPi(rbt.model.ikcon(T));%,armManipulate));
            % qT = jtraj(q,q2,steps);

            q = rbt.model.getpos();
            R = eye(3);  % Identity rotation matrix
            T = transl(objPos) * [R, [0; 0; 0]; 0 0 0 1];  % Homogeneous transformation
            q2 = wrapToPi(rbt.model.ikcon(T));
            qT = jtraj(q, q2, steps);
        end
        function qT = CreateTrajectoryShaking(rbt,objPos,steps)
            %steps = 200;%100
            q = rbt.model.getpos();
            T = transl(objPos)*trotx(pi)*troty(0)*trotz(0);
            % T = transl(objPos+[0,0,0.2])*trotx(pi)*troty(0)*trotz(0);
            q2 = wrapToPi(rbt.model.ikcon(T));%,armManipulate));
            qT = jtraj(q,q2,steps);
        end

        function  moveToPos(rbt,qTj,finger,mfinger)


            q_f1_end = deg2rad([25, 0]);
            q_f2_end = deg2rad([25, 0]);

            % Get the initial positions for both fingers
            q_f1_start = finger.model.getpos();
            q_f2_start = mfinger.model.getpos();

            % Generate the joint trajectories for each finger
            q_f1_traj = jtraj(q_f1_start, q_f1_end, size(qTj, 1));
            q_f2_traj = jtraj(q_f2_start, q_f2_end, size(qTj, 1));

            % Loop over the main trajectory
            for i = 1:size(qTj, 1)
                q = qTj(i, :);

                % Animate the robot model
                rbt.model.animate(q);

                % Compute the base transformation for the gripper fingers
                base = rbt.model.fkineUTS(q);
                % baseTransform = base * transl(0, 0, 0.1) * trotz(pi/2);

                % Set the transformed base for each finger and animate
                finger.model.base = base;% * trotx(pi/2);
                finger.model.animate(q_f1_traj(i, :));

                mfinger.model.base = base * trotz(pi);% * troty(pi) * trotx(-pi/2);
                mfinger.model.animate(q_f2_traj(i, :));

                % Pause to create a smooth animation
                pause(0.02);

            end
        end


        function finalShakerPosition = PlotShakerShaking(rbt, qTj,shakerHand)
            % Read the shaker mesh data in
            [f, v, ~] = plyread('ShakerBody.ply', 'tri');

            shakerMesh_h = []; % Initialize the handle for the shaker mesh
            finalShakerPosition = [];
            q_shaker_end = deg2rad([25, 0]);

            % Get the initial positions of the shaker hand
            q_shaker_start = shakerHand.model.getpos();

            % Generate joint trajectory for shaker hand movement
            % q_shaker_traj = jtraj(q_shaker_start, size(qTj, 1));
            q_shaker_traj = jtraj(q_shaker_start, q_shaker_end, size(qTj, 1));


            for i = 1:size(qTj, 1)
                q = qTj(i, :);
                rbt.model.animate(q);
                base = rbt.model.fkineUTS(q);
                % Set the transformed base for each finger and animate
                shakerHand.model.base = base;
                shakerHand.model.animate(q_shaker_traj(i, :));

                % Pause to create a smooth animation
                pause(0.02);
                %     for i = 1:size(qTj,1)
                %         q = qTj(i,:);
                %         rbt.model.animate(q);
                %
                %         pause(0.05)
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


        function  moveToPosShaking(rbt,qTj,shakerHand)


            q_shaker_end = deg2rad([25, 0]);

            % Get the initial positions of the shaker hand
            q_shaker_start = shakerHand.model.getpos();

            % Generate joint trajectory for shaker hand movement
            % q_shaker_traj = jtraj(q_shaker_start, size(qTj, 1));
            q_shaker_traj = jtraj(q_shaker_start, q_shaker_end, size(qTj, 1));

            % Loop over the main trajectory
            for i = 1:size(qTj, 1)
                q = qTj(i, :);

                % Animate the robot model
                rbt.model.animate(q);

                % Compute the base transformation for the gripper fingers
                base = rbt.model.fkineUTS(q);
                % baseTransform = base * transl(0, 0, 0.1) * trotz(pi/2);

                % Set the transformed base for each finger and animate

                shakerHand.model.base = base;
                shakerHand.model.animate(q_shaker_traj(i, :));



                % Pause to create a smooth animation
                pause(0.02);

            end
        end

    end

end



% function  moveToPos(rbt,qTj)
%
%
%     for i = 1:size(qTj,1)
%         q = qTj(i,:);
%         rbt.model.animate(q);
%
%         pause(0.05)
%     end
%
% end
%         function moveToPos(rbt, qTj)
%     % Define the initial base transformation for the fingers
%     fingerBaseTransform = transl(0, 0, 0.1) * trotz(pi/2);
%
%     % Initial positions for fingers, assuming rbt.model.fkine(qTj(1,:)) is a 4x4 matrix
%     finger = RG2Finger(eye(4));  % Initialize with identity to avoid transformation conflicts
%     mFinger = RG2Finger(eye(4));
%
%     for i = 1:size(qTj, 1)
%         q = qTj(i, :);
%         rbt.model.animate(q);
%
%         % Calculate current end-effector transformation matrix
%         endEffectorTransform = rbt.model.fkine(q);
%
%         % Apply the base transformations to the fingers
%         finger.model.base = endEffectorTransform * fingerBaseTransform;
%         mFinger.model.base = endEffectorTransform * fingerBaseTransform * trotz(pi);
%
%         % Animate fingers at their updated positions
%         finger.model.animate(0);  % Single joint update if fingers are prismatic
%         mFinger.model.animate(0);
%
%         pause(0.05);
%     end
% end