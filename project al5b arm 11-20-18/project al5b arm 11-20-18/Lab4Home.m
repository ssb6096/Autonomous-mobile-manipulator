% Create the new ArmRobot object
robot = ArmRobot('COM5');

% values used Throughout
joints = [0 1 2 3 4 5];
%starting step to find center
testc = [1500 1500 1500 1500 1500 1500];
%testing base
testbasec=[1500 1500 1500 1500 1500 1500];
testbasel=[650 1500 1500 1500 1500 1500];
testbaser=[2300 1500 1500 1500 1500 1500];
%testing shoulder
testshoulderc=[1500 1900 1500 1500 1500 1500];
%testing elbow
testelbowc=[1500 1600 1500 1500 1500 1500];
%testing wrist
testwristc=[1500 1600 1500 1500 1500 1500];
testwristtwc1=[1500 1600 1500 1500 1500 1500];
testwristtwc=[1500 1600 1500 1500 1200 1200];
%testing grip
testgrip=[1500 1600 1500 1500 1500 1200];
opengrip=[1500 1600 1500 1500 1500 900];
%trying to open bottle
position1 = [1500 1900 1500 1500 1500 1500];
position2 = [1500 1650 1450 600 1500 1100];
%twisting the cap
position3 = [1500 1650 1450 600 1400 100];
position4 = [1500 1650 1450 600 1300 1100];
position5 = [1500 1650 1450 600 1200 1100];
position6 = [1500 1650 1450 600 1200 1200];

opengrip1 = [1500 1650 1450 600 700 900];
position7 = [1500 1650 1450 600 700 1200];
position8 = [1500 1650 1450 600 2300 1200];
position9 = [1500 1650 1450 600 2300 900];
position10 = [1500 1650 1450 600 700 900];
position11 = [1500 1650 1450 600 700 1300];
position12 = [1500 1850 1450 600 700 1300];

opengrip1 = [1500 1650 1450 600 1500 900];


balancewrist=[1500 1800 1500 1500 1200 1200];
% center=[1450 1500 1400 1475 1450 2075];

% values used for Repeatability
%pencilTap = [1450 1500 2000 2025 1450 2075];
%centerGrip = [1450 1500 1400 1475 1480 2075];

% values used for Absolute move
% centerD = [0 0 0 0 0 0];
% toTheLeft = [-60 0 0 0 0 0];
% toTheRight = [ 0 0 30 0 0 0];

% values used for Relative move
% global newAngle;
% centerR = [0 0 0 0 0 0];
% moveLeft = [5 0 0 0 0 0];

% Set configuration of the robot
robot.setServoCenters([1450 1500 1400 1475 1480 1575]);
robot.setServoBounds([555 800 900 575 575 900],[2350 2150 2000 2400 2350 2250]);
robot.setLinkLengths([3.66 4.2 1.69 10.69 11.25 6.19 4.78]);

% Connect to the Robot
robot.connect();

% Step 0 - Home Position
%display('Press Any Key to go to home position');
%for i = 0:1:10
% robot.moveJoints(center,joints);
% pause(2);

% robot.moveJoints(testc,joints);
% pause(2);
% robot.moveJoints(opengrip,joints);
% pause(4);
% robot.moveJoints(position1,joints);
% pause(4);
robot.moveJoints(opengrip1,joints);
pause(5);
for i = 1:1:7
robot.moveJoints(position7,joints);
pause(1);
robot.moveJoints(position8,joints);
pause(1);
robot.moveJoints(position9,joints);
pause(1);
robot.moveJoints(position10,joints);
pause(1);
end

robot.moveJoints(position11,joints);

robot.moveJoints(position12,joints);
% robot.moveJoints(balancewrist,joints);
% pause(2);

%pause(2);
%
% Step 1 - Center Grip
% display('Press Any Key for Step 2');
%robot.moveJoints(centerGrip,joints);
%robot.moveJoints(pencilTap,joints);
%end
%pause(2);
% 
% % Step 2 - Absolute
% display('Press Any Key for Step 3');
%robot.absoluteJointMove(centerD,joints);
%pause(2);
%robot.absoluteJointMove(toTheLeft,joints);
%pause(2);
%robot.absoluteJointMove(centerD,joints);
%pause(2);
%robot.absoluteJointMove(toTheRight,joints);
%pause(2);
%robot.absoluteJointMove(centerD,joints);
%pause(2);
% 
% % Step 3 - Relative
% display('Press Any Key for Step 4');
% newAngle = center;
% newAngle = robot.relativeJointMove(centerR,joints,newAngle);
% newAngle = robot.relativeJointMove(centerR,joints,newAngle);
% pause();
% 
%  
% % Step 4
% display('Press Any Key for Step 5');
% pause();
%  
% % Step 5
% display('Press Any Key for Step 6');
% pause();
%  
% % Step 6
% display('Press Any Key to all steps absolute');
% pause();
% 
% % Now Lets Try that All At Once (Absolute)
% display('Press Any Key to all steps relative');
% pause();
% 
% % Now Lets Try that All At Once (Relative)
% display('Press Any Key to all steps relative linear');
% pause();
% 
% % Now Lets Try that SMOOTHLY All At Once (Relative Linear)
% display('Press Any Key to all steps absolute linear');
% pause();

% Close out the robot

robot.delete();
