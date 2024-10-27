clear all;
clf
clc;

%initiallise classes
env=Environment();% not really needed

%% Set Positions
% adjustment varibale
sx = -0.5; %variables for shaker position
sy = -0.6;
sz = 0.9;
UR_OS1 = 0.3; %UR10 offset variable
Jak_OS1 =1.0; %Jaka Zu offset

%Environment Vairables
tablePOS=[-0.4,0,0];
Bottle1POS=[0, 0.8, 2];
Bottle2POS=[-0.5, 0.8, 2];
Bottle3POS=[-1,0.8,2];
Button1POS = [-0.05, 0.8, 1.35];
Button2POS = [-0.55, 0.8, 1.35];
Button3POS = [-1.05, 0.8, 1.35];
ShakerPOS=[sx,sy,sz];
% workspace = [-2.5 2.5 -2 2 -0.02 5];
% scale =0.1;
EStopPOS=[-2.15, -0.8, 1];
FextPOS=[-2,-1.3,0.5];

%Robot Pos
ShakerGrabPOS=[sx,sy,sz+UR_OS1];
Mid1POS = [-0.30, 0.6, 1.35]; %positions between buttons
Mid2POS = [-0.80, 0.6, 1.35];
UR_default = [0, 0, 1.5]
HandoffPos = [0.16, 0 , 1.96];
PourPos = [0.2, -2, 1.2]

JakPos1 = [0.4,-0.22,2.43]
JakPos2 = [-0.4,-0.22,2.43]
%%  Plot environment.
% [p1,s1,b1,b2,b3,b4,b5,b6,e1,f1] = env.simEnvironment(tablePOS,ShakerPOS,Bottle1POS,Bottle2POS,Bottle3POS,Button1POS,Button2POS,Button3POS,EStopPOS,FextPOS);
[p1,s1,b1,b2,b3,b4,b5,b6,e1,f1] = Environment.simEnvironment(tablePOS,ShakerPOS,Bottle1POS,Bottle2POS,Bottle3POS,Button1POS,Button2POS,Button3POS,EStopPOS,FextPOS);
drawnow();
hold on


%%  Plot Jaka Zu 3 Robot 2
% robot1 = UR3e();  % Initialize robot1 as UR3e
robot2 = JakaZu3(transl(0,0,3)*troty(pi));  % Initialize robot1 as Jaka Zu 3
% Initialize q_init for robot1
% q_init_robot1 = zeros(robot1.model.n);  % Joint configuration for robot1 as a column vector
% R1 = troty(pi);  % Rotation matrix for 180 degrees around y-axis
% T1 = transl(0, 0, 3);  % Translation matrix (up 3.5 units along z-axis)
% baseTransform1 = T1 * R1;
% robot1.model.base = baseTransform1;
hold on;
% axis(workspace);
% robot1.model.plot(q_init_robot1, 'workspace', workspace, 'scale', scale);  % Ensure this is a column vector
%%  Plot LinearUR10 Robot 1
robot1 = LinearUR10(transl(0,0,0.5));  % Initialize robot2 as LinearUR10
hold on;

qC = robot1.model.getpos()
% TC = transl(eye(4))*trotx(pi)*troty(0)*trotz(0)
base = robot1.model.fkineUTS(qC)
% base2 = robot1.model.fkineUTS(qC)
% base = wrapToPi(robot1.model.ikcon(TC));

% Define a base transformation with a translation and rotation
baseTransform = base * transl(0, 0, 0.1) * trotz(pi/2);

% Apply transformations directly to both fingers
finger = RG2Finger(baseTransform);                     % Transformation for finger1
mFinger = RG2Finger(baseTransform * trotz(pi));         % Transformation for finger2, rotated 180° around z-axis

% % Take the base of end-effector 
% qNow = r.model.getpos();
% base = r.model.fkineUTS(qNow);
% 
% 
% %Call the gripper with 2 fingers and plot the calculated base 
% finger1 = LinearFinger(base*trotx(pi/2)); 
% 
% finger2 = LinearFinger(base*troty(pi)*trotx(-pi/2));


%% Ikcon values to manipulate the arms position
% toShakerAvoidTable = deg2rad([0 0 0 -90 0 90 0]);

%% Move to Shaker
qTj = Control.CreateTrajectory(robot1,ShakerGrabPOS);%,toShakerAvoidTable);
% Control.moveToPos(robot1,qTj);
Control.moveToPos(robot1,qTj,finger,mFinger);

%% Grab Shaker and move to default position
% Calculate move to Standing Position
qi = [-0.55, -0.5, 2];
qTj = Control.CreateTrajectory(robot1, qi);

%Delete the initial brick
try delete(s1);
catch ME
end

s1NewPos = Control.PlotShaker(robot1,qTj);
s1 = PlaceObject('Shaker.ply', transl(s1NewPos));
% 
% %Place brick
% 
% qTj = Control.CreateTrajectory(r, Button1POS);
% %Delete the initial brick
% try delete(b1);
% catch ME
% end
% 
% RobotMoveBrick.PlotBrick(r,qTj);
% b1 = PlaceObject('HalfSizedRedGreenBrick.ply', brick1EndPos);
% 
% qf = [-0.2, 0, 2];
% % Move to Standing Position
% qTj = RobotTracjectory.CreateTrajectory(r, qf);
% RobotToBrick.moveToBrickNoPos(r,qTj);


%% Move to Drink/Button 1
qTj = Control.CreateTrajectory(robot1, Button1POS);
Control.moveToPos(robot1,qTj);



