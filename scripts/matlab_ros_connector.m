%% Initialize ROS-Matlab connection
% Define ROS master's IP address and port
ipAddress = 'http://localhost:11311';

% Initialize ROS master's connection
rosinit(ipAddress);

%% Create subscriber to listen the mission over signal
mission_over_sub = rossubscriber('/mission_over', 'mission_planner/MissionOver');

% Receive a message from mission over subscriber, waiting up to 10s
mission_over = receive(mission_over_sub, 10);
% Show the message information
disp(mission_over.value);

%% End connection with ROS
rosshutdown;
