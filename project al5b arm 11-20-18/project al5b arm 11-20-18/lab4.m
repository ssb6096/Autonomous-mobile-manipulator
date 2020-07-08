% Create the new ArmRobot object
robot = ArmRobot('COM5');

% Set configuration of the robot
robot.setServoCenters([1330 1550 1770 1600 1640 1500]);
robot.setServoBounds([500 750 1000 700 750 1100],[2250 2050 2450 2450 2500 2500]);
robot.setLinkLengths([5 5 2 12 13.5 6 6.5]);

% Connect to the Robot
robot.connect();

display('Press Any Key to go to home position');
% 
% Step 0 - Home Position 

% % Step 1
% display('Press Any Key for Step 2');
% pause();
% 
% % Step 2
% display('Press Any Key for Step 3');
% pause();
% 
% % Step 3
% display('Press Any Key for Step 4');
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
