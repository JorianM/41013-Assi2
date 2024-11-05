% clear all;
% clf;
% clc;

%initiallise classes d
% env=Environment();% not really needed

%% Set Positions
% adjustment varibale
global currentValue;
sx = -0.5; %variables for shaker position
sy = -0.6;
sz = 0.9;
UR_OS1 = 0.3; %UR10 offset variable
Jak_OS1 =1.0; %Jaka Zu offset

steps_long=60;
steps_short=30;
v_short=20;
%Environment Vairables

tablePOS=[-0.4,0,0];
Bottle1POS=[0, 0.8, 2];
Bottle2POS=[-0.5, 0.8, 2];
Bottle3POS=[-1,0.8,2];
Button1POS = [-0.05, 0.8, 1.35];
Button2POS = [-0.55, 0.8, 1.35];
Button3POS = [-1.05, 0.8, 1.35];

workerPos = [2.3,0,0];
targetPos = [1, 0, 0.5]; % Target position
workerSpeed = 0.1; % Speed of the worker's movement
flag = false; % Control flag for movement

LightCurt1POS = [1.7, -1, 1];%z=1.1
LightCurt1POS1 = [1.6, -0.98, 0.8];
LightCurt1POS2 = [-1.6, -1.12, 2.5];
LightCurt1RbtKill = [0, -1.2, 1.18];%1.2


LightCurt2POS = [2.05, -0.3, 0];
LightCurt2POS1 = [2.05, -0.3, 0];
LightCurt2POS2 = [2.045, 0.5, 3.5];


GlassPOS= [-2, 0, 1.1];
ShakerPOS=[sx,sy,sz];
% workspace = [-2.5 2.5 -2 2 -0.02 5];
% scale =0.1;
EStopPOS=[-2.15, -0.8, 1];
FextPOS=[-2,-1.3,0.5];

%Robot Pos
% ShakerGrabPOS=[sx+0.2,sy,sz+0.1];
% ShakerGrabPOS=[sx,sy,sz];
Mid1POS = [-0.30, 0.6, 1.35]; %positions between buttons
Mid2POS = [-0.80, 0.6, 1.35];
UR_default = [-1, 0, 1.35];
StandPOS = [-0.55, 0, 1.35];
HandoffPos = [0.16, 0 , 1.96];
PourPos = [0.2, -2, 1.2];
JakPos0 = [0.16,0,2.4];
JakPos1 = [0.4,-0.22,2.43];
JakPos2 = [-0.4,-0.22,2.43];


%%  Plot environment.
% [p1,s1,b1,b2,b3,b4,b5,b6,e1,f1] = env.simEnvironment(tablePOS,ShakerPOS,Bottle1POS,Bottle2POS,Bottle3POS,Button1POS,Button2POS,Button3POS,EStopPOS,FextPOS);
[p1,s1,b1,b2,b3,b4,b5,b6,L1,L2,g1,e1,f1] = Environment.simEnvironment(tablePOS,ShakerPOS,Bottle1POS,Bottle2POS,Bottle3POS,Button1POS,Button2POS,Button3POS,LightCurt1POS,LightCurt2POS,GlassPOS,EStopPOS,FextPOS);
drawnow();
hold on


%%  Plot Jaka Zu 3 Robot 2
% robot1 = UR3e();  % Initialize robot1 as UR3e
robot2 = JakaZu3(transl(0,0,3)*troty(pi));  % Initialize robot1 as Jaka Zu 3

hold on;
qC = robot2.model.getpos();
base = robot2.model.fkineUTS(qC);
baseTransform = base * trotz(pi/2);
shakerHand = ShakerHand(baseTransform);


%%  Plot LinearUR10 Robot 1
workspace =[-2.5 2.5 -2 2 -0.02 5];
            axis(workspace);
            view([43.5,17.35]);
            camzoom(1);
robot1 = LinearUR10(transl(0,0,0.5));  % Initialize robot2 as LinearUR10
hold on;

qC = robot1.model.getpos();
base = robot1.model.fkineUTS(qC);
% test = base(1,4)
% % Define a base transformation with a translation and rotation
% % baseTransform = base * transl(0, 0, 0.1) * trotz(pi/2);
baseTransform = base * trotz(pi/2);
% 
% % Apply transformations directly to both fingers
finger = RG2Finger(baseTransform);                     % Transformation for finger1
mfinger = RG2Finger(baseTransform * trotz(pi));         % Transformation for finger2, rotated 180° around z-axis

%% Ikcon values to manipulate the arms position
% toShakerAvoidTable = deg2rad([0 0 0 -90 0 90 0]);


%% Initialize Arduino E-stop
% % comPort = "/dev/cu.usbmodem14101"; % Adjust to your Arduino COM port
% % boardType = 'Uno'; % Change if you're using a different board
% % buttonPin = 'D2'; % Pin for the pushbutton
% % ledPin = 'D13'; % Pin for the LED
% % arduinoEStop = ArduinoTest(comPort, boardType, buttonPin, ledPin);
% % arduinoEStop.startMonitoring(); % Start monitoring the button
% % 
% % pause(100);
% 
% comPort = "/dev/cu.usbmodem14101"; % Adjust to your Arduino COM port
% baudRate = 9600; % Set the baud rate (must match Arduino sketch)
% %global stopFlag;
% %stopFlag = False;
% % Create an instance of the ArduinoTest class
% ard = ArduinoTest(comPort, baudRate);
% 
% 
% pause(30);

%% Light curtain 
Control.lightCurtain(robot1,LightCurt1RbtKill, steps_long, finger, mfinger, LightCurt1POS1, LightCurt1POS2);


%% Move to Shaker
qTj = Control.CreateTrajectory(robot1,ShakerPOS,steps_long);%,toShakerAvoidTable);
% Control.moveToPos(robot1,qTj);
Control.moveToPos(robot1,qTj,finger,mfinger);
pause(0.01);
% qC = robot1.model.getpos()
% base = robot1.model.fkineUTS(qC)
%% Grab Shaker and move to default position
% Calculate move to Standing Position
qTj = Control.CreateTrajectory(robot1, StandPOS,steps_long);
%Delete the initial brick
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);

%% Move to Drink/Button 1
qTj = Control.CreateTrajectory(robot1, Button1POS,steps_long);
%Control.moveToPos(robot1,qTj,finger,mfinger);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
% s1 = PlaceObject('ShakerBody.ply', transl(Button1Pos));
%% Move to midpoint1
qTj = Control.CreateTrajectory(robot1, Mid1POS,steps_short);
%Control.moveToPos(robot1,qTj,finger,mfinger);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move to Button2
qTj = Control.CreateTrajectory(robot1, Button2POS,steps_short);
%Control.moveToPos(robot1,qTj,finger,mfinger);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move to Midpoint2
qTj = Control.CreateTrajectory(robot1, Mid2POS,steps_short);
%Control.moveToPos(robot1,qTj,finger,mfinger);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move to Button 3
qTj = Control.CreateTrajectory(robot1, Button3POS,steps_short);
%Control.moveToPos(robot1,qTj,finger,mfinger);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move back to midpoint
qTj = Control.CreateTrajectory(robot1, UR_default,steps_long);
%Control.moveToPos(robot1,qTj,finger,mfinger);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move to Handoff
qTj = Control.CreateTrajectory(robot1, HandoffPos,steps_short);
%Control.moveToPos(robot1,qTj,finger,mfinger);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);

%% Move to Handoff R2
qTj = Control.CreateTrajectoryShaking(robot2, JakPos0,steps_short);
Control.moveToPosShaking(robot2,qTj,shakerHand);
pause(0.01);
%% Move shaker Robot 2
qTj = Control.CreateTrajectoryShaking(robot2, JakPos1,steps_short);

% Control.moveToPos(robot1,qTj);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShakerShaking(robot2,qTj,shakerHand);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move shaker Robot 2
qTj = Control.CreateTrajectoryShaking(robot2, JakPos2,steps_short);

% Control.moveToPos(robot1,qTj);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShakerShaking(robot2,qTj,shakerHand);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move shaker Robot 2
qTj = Control.CreateTrajectoryShaking(robot2, JakPos1,steps_short);

% Control.moveToPos(robot1,qTj);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShakerShaking(robot2,qTj,shakerHand);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move shaker Robot 2
qTj = Control.CreateTrajectoryShaking(robot2, JakPos2,steps_short);

% Control.moveToPos(robot1,qTj);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShakerShaking(robot2,qTj,shakerHand);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);
%% Move to Handoff R2
qTj = Control.CreateTrajectoryShaking(robot2, JakPos0,steps_short);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShakerShaking(robot2,qTj,shakerHand);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);

%% Move to glass
qTj = Control.CreateTrajectory(robot1, GlassPOS,steps_short);
%Control.moveToPos(robot1,qTj,finger,mfinger);
try delete(s1);
catch ME
end
s1NewPos = Control.PlotShaker(robot1,qTj,finger,mfinger);
s1 = PlaceObject('ShakerBody.ply', transl(s1NewPos));
pause(0.01);

%%