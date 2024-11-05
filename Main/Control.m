classdef Control < handle
    properties
    end
    methods (Static)

        function finalShakerPosition = PlotShaker(rbt, qTj,finger,mfinger)
            global currentValue;
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
                % Pause if currentValue is 1
                if (any(currentValue == 1)) && (i ~= 1)
                    i = i - 1; % Stay at the current step, ensuring i >= 1
                    pause(0.1);        % Small pause to reduce CPU load
                    continue;          % Skip the rest of this loop iteration
                end
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
                % startWorkerMovement();
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
        function qT = CreateTrajectoryShaking(rbt,objPos,steps)% will remove probs
            %steps = 200;%100
            q = rbt.model.getpos();
            T = transl(objPos)*trotx(pi)*troty(0)*trotz(0);
            % T = transl(objPos+[0,0,0.2])*trotx(pi)*troty(0)*trotz(0);
            q2 = wrapToPi(rbt.model.ikcon(T));%,armManipulate));
            qT = jtraj(q,q2,steps);
        end

        function  moveToPos(rbt,qTj,finger,mfinger)
            global currentValue;
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
                % Pause if currentValue is 1
                if (any(currentValue == 1)) && (i ~= 1)
                    i = i - 1; % Stay at the current step, ensuring i >= 1
                    pause(0.1);        % Small pause to reduce CPU load
                    continue;          % Skip the rest of this loop iteration
                end
                q = qTj(i, :);
                % Animate the rbt model
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
            global currentValue;
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
                % Pause if currentValue is 1
                if (any(currentValue == 1)) && (i ~= 1)
                    i = i - 1; % Stay at the current step, ensuring i >= 1
                    pause(0.1);        % Small pause to reduce CPU load
                    continue;          % Skip the rest of this loop iteration
                end
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
            global currentValue;


            q_shaker_end = deg2rad([25, 0]);

            % Get the initial positions of the shaker hand
            q_shaker_start = shakerHand.model.getpos();

            % Generate joint trajectory for shaker hand movement
            % q_shaker_traj = jtraj(q_shaker_start, size(qTj, 1));
            q_shaker_traj = jtraj(q_shaker_start, q_shaker_end, size(qTj, 1));

            % Loop over the main trajectory
            for i = 1:size(qTj, 1)
                % Pause if currentValue is 1
                if (any(currentValue == 1)) && (i ~= 1)
                    i = i - 1; % Stay at the current step, ensuring i >= 1
                    pause(0.1);        % Small pause to reduce CPU load
                    continue;          % Skip the rest of this loop iteration
                end
                q = qTj(i, :);

                % Animate the rbt model
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


        function lightCurtain(rbt,objPos,steps, finger, mfinger, LightCurtPOS1,LightCurtPOS2)%workerPos, workerSpeed, targetPos)
            global currentValue;
            % Put into enviro
            [v, f, fn] = Control.RectangularPrism(LightCurtPOS1,LightCurtPOS2);%[2, -1.1, -1], [3, 1.1, 1]);

            % Create Traj
            q = rbt.model.getpos();
            q1 = q;
            T = transl(objPos)*trotx(pi)*troty(0)*trotz(0);
            % T = transl(objPos+[0,0,0.2])*trotx(pi)*troty(0)*trotz(0);
            q2 = wrapToPi(rbt.model.ikcon(T));%,armManipulate));
            qTj = jtraj(q1,q2,steps);

            q_f1_end = deg2rad([25, 0]);
            q_f2_end = deg2rad([25, 0]);
            % Get the initial positions for both fingers
            q_f1_start = finger.model.getpos();
            q_f2_start = mfinger.model.getpos();
            % Generate the joint trajectories for each finger
            q_f1_traj = jtraj(q_f1_start, q_f1_end, size(qTj, 1));
            q_f2_traj = jtraj(q_f2_start, q_f2_end, size(qTj, 1));

            %
            collisionFound = false;
            % for step = 1:length(qTj)
            for i = 1:steps
                % Pause if currentValue is 1
                if (any(currentValue == 1)) && (i ~= 1)
                    i = i - 1; % Stay at the current step, ensuring i >= 1
                    pause(0.1);        % Small pause to reduce CPU load
                    continue;          % Skip the rest of this loop iteration
                end
                q = qTj(i, :);
                base = rbt.model.fkineUTS(q);
                finger.model.base = base;% * trotx(pi/2);
                finger.model.animate(q_f1_traj(i, :));
                mfinger.model.base = base * trotz(pi);% * troty(pi) * trotx(-pi/2);
                mfinger.model.animate(q_f2_traj(i, :));
                % Pause to create a smooth animation
                pause(0.02);
                rbt.model.animate(q);
                %ende
                % endETestx = base(1,4);
                % endETesty = base(2,4);
                % endETestz = base(3,4);
                %hand
                endETestx = base(1,4);
                endETesty = base(2,4);
                endETestz = base(3,4);

                % if (endETestx < 1.6) && (endETestx > -1.6) && (endETesty <-0.98) &&(endETesty > -1.12)
                if (endETestx < LightCurtPOS1(1,1)) && (endETestx > LightCurtPOS2(1,1)) && (endETesty < LightCurtPOS1(1,2)) &&(endETesty > LightCurtPOS2(1,2)) && (endETestz > LightCurtPOS1(1,3)) &&(endETestz < LightCurtPOS2(1,3))
                    % % Calculate end effector position based on joint angles
                    % endEffectorPos =  rbt.model.fkineUTS(q) % Implement this function

                    % if Control.IsCollision(rbt, qTj(i, :), f, v, fn, false)
                    disp(['Collision detected at step ' num2str(i)]);
                    % disp(['Pose at step ' num2str(i) ' is: q = [' num2str(qTj(i,1)) ' ' num2str(qMatrix(i,2)) ' ' num2str(qMatrix(i,3)) ']']);
                    collisionFound = true;
                    break;
                end

                % end
            end


            if ~collisionFound
                disp('No collision detected in the trajectory.');
            end
        end


        %% Rectangle
        function [vertex,face,faceNormals] = RectangularPrism(lower,upper,plotOptions,axis_h)
            if nargin<4
                axis_h=gca;
                if nargin<3
                    plotOptions.plotVerts=false;
                    plotOptions.plotEdges=true;
                    plotOptions.plotFaces=true;
                end
            end
            hold on

            vertex(1,:)=lower;
            vertex(2,:)=[upper(1),lower(2:3)];
            vertex(3,:)=[upper(1:2),lower(3)];
            vertex(4,:)=[upper(1),lower(2),upper(3)];
            vertex(5,:)=[lower(1),upper(2:3)];
            vertex(6,:)=[lower(1:2),upper(3)];
            vertex(7,:)=[lower(1),upper(2),lower(3)];
            vertex(8,:)=upper;

            face=[1,2,3;1,3,7;
                1,6,5;1,7,5;
                1,6,4;1,4,2;
                6,4,8;6,5,8;
                2,4,8;2,3,8;
                3,7,5;3,8,5;
                6,5,8;6,4,8];

            if 2 < nargout
                faceNormals = zeros(size(face,1),3);
                for faceIndex = 1:size(face,1)
                    v1 = vertex(face(faceIndex,1)',:);
                    v2 = vertex(face(faceIndex,2)',:);
                    v3 = vertex(face(faceIndex,3)',:);
                    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
            end
            %% If plot verticies
            if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
                for i=1:size(vertex,1);
                    plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
                    text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
                end
            end

            %% If you want to plot the edges
            if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
                links=[1,2;
                    2,3;
                    3,7;
                    7,1;
                    1,6;
                    5,6;
                    5,7;
                    4,8;
                    5,8;
                    6,4;
                    4,2;
                    8,3];

                for i=1:size(links,1)
                    plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
                        [vertex(links(i,1),2),vertex(links(i,2),2)],...
                        [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
                end
            end

            %% If you want to plot the edges
            if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
                tcolor = [.2 .2 .8];

                patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','lineStyle','none');
            end

        end
        
    end


end


