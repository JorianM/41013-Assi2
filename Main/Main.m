clf
clc
%initiallise classes
env=Environment();

%Load variables
tablePOS=[-0.4,0,0];
ShakerPOS=[0,-0.6,0.9];
Bottle1POS=[0, 0.8, 2];
Bottle2POS=[-0.5, 0.8, 2];
Bottle3POS=[-1,0.8,2];
Button1POS = [-0.05, 0.8, 1.35];
Button2POS = [-0.55, 0.8, 1.35];
Button3POS = [-1.05, 0.8, 1.35];
workspace = [-2.5 2.5 -2 2 -0.02 5];
scale =0.1;
%Place Robots
        robot1 = UR3e();  % Initialize robot1 as UR3e
        robot2 = LinearUR10(transl(0,0,0.5));  % Initialize robot2 as LinearUR10
        
        % Initialize q_init for robot1
        q_init_robot1 = zeros(robot1.model.n);  % Joint configuration for robot1 as a column vector
        R1 = troty(pi);  % Rotation matrix for 180 degrees around y-axis
        T1 = transl(0, 0, 3);  % Translation matrix (up 3.5 units along z-axis)
        baseTransform1 = T1 * R1;
        robot1.model.base = baseTransform1;

        hold on;
        axis(workspace);
        robot1.model.plot(q_init_robot1, 'workspace', workspace, 'scale', scale);  % Ensure this is a column vector
        

%load in environment.
        [p1,s1,b1,b2,b3,b4,b5,b6] = env.simEnvironment(tablePOS,ShakerPOS,Bottle1POS,Bottle2POS,Bottle3POS,Button1POS,Button2POS,Button3POS);