%% Initialize ROS-Matlab connection
% Define ROS master's IP address and port
ip_address = 'http://localhost:11311';

% Initialize ROS master's connection
rosinit(ip_address);

%% Subscribers
% Create subscriber to listen the mission over signal
mission_over_sub = rossubscriber('/mission_over', 'mission_planner/MissionOver');

%% Action servers
% Create action server to answer planning/replanning requests
[planning_server, planning_goal_msg] = rosactionserver('/heuristic_planning', 'mission_planner/HeuristicPlanning', @heuristicPlanningCallback);

% TODO: Create action server to answer repair requests

% Wait until the action client is connected
disp('Waiting for high level planner connection...');
waitForServer(planning_server);

% Initialize loop variables
ros_ok = true;
mission_over = false;

% Run while ros::ok() or mission not over
while ros_ok && not(mission_over)
    % Check if ros is still okay
    try
        rosnode list;
    catch
        ros_ok = false;
    end

    % Check if mission is over
    % Receive a message from mission over subscriber, waiting up to 0.1s
    % mission_over = receive(mission_over_sub, 0.1);
    if not(isempty(mission_over_sub.LatestMessage)) && mission_over_sub.LatestMessage.Value == true
        mission_over = true;
    end
end

%% End connection with ROS
rosshutdown;

function heuristicPlanningCallback(~, goal_msg, default_goal_handle)
    disp('Received a new planning request')

    % Set up the feedback and the result
    feedback_msg = rosmessage('mission_planner/HeuristicPlanningFeedback');
    result_msg = rosmessage('mission_planning/HeuristicPlanningResult');

    % Send feedback to the client
    sendFeedback(planning_server, feedback_msg, default_goal_handle);

    % Return the final result to the client
    result_msg = true;
    setResult(planning_server, result_msg, default_goal_handle);
end