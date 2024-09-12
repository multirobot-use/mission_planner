%% Initialize ROS-Matlab connection
% Define ROS master's IP address and port
ip_address = 'http://localhost:11311';

% Check if ROS master's connection is alive
try
    rosnode list;
catch
    % Initialize ROS master's connection
    rosinit(ip_address);
end

% Read the mission id from the config file
config_file_path = '../config/conf.yaml';
config_file = fopen(config_file_path, 'r');
scenario_id = fscanf(config_file, 'mission_id: %s');

% Load scenario's data
[Agent, Task] = scenario(scenario_id);

%% Node
% Create a ROS node
if ~exist('node', 'var')
    node = ros.Node('/matlab_ros_connector');
end

%% Subscribers
% Create subscriber to listen the mission over signal
mission_over_sub = ros.Subscriber(node, '/mission_over', 'mission_planner/MissionOver');

%% Action servers
% Create action server to answer planning/replanning requests
planning_server = ros.SimpleActionServer(node, '/heuristic_planning', 'mission_planner/HeuristicPlanning', ...
    ExecuteGoalFcn=@heuristicPlanningCallback, DataFormat="struct");
    % ExecuteGoalFcn={@heuristicPlanningCallback, Agent, Task}, DataFormat="struct");

disp('Heuristic planning action server is ready');

% TODO: Create action server to answer repair requests

% Wait until the action client is connected
% disp('Waiting for high level planner connection...');
% waitForServer(planning_server);

% Initialize loop variables
ros_ok = true;
mission_over = false;

% Run while ros::ok() or mission not over
while ros_ok && not(mission_over)
    % Check if ros is still okay
    % try
    %     rosnode list;
    % catch
    %     ros_ok = false;
    % end

    % Check if mission is over
    % Receive a message from mission over subscriber, waiting up to 0.1s
    % mission_over = receive(mission_over_sub, 0.1);
    if not(isempty(mission_over_sub.LatestMessage)) && mission_over_sub.LatestMessage.Value == true
        mission_over = true;
    end
end

%% End connection with ROS
rosshutdown;

%function [result, success] = heuristicPlanningCallback(src, goal_msg, feedback_msg, result_msg, Agent, Task)
function [result, success] = heuristicPlanningCallback(src, goal_msg, feedback_msg, result_msg)%, Agent, Task)
    % Inputs:
    % src is the associated action server object
    % goal_msg is the goal message sent by the action client
    % feedback_msg is the response message (feedback)
    % result_msg is the result message
    % Outputs:
    % result is a action result message
    % success is a bool variable, true if the goal was reached, false if the goal was aborted or preempted

    disp('Received a new planning request')

    % Set up the feedback and the result
    feedback_msg = rosmessage('mission_planner/HeuristicPlanningFeedback');
    result_msg = rosmessage('mission_planning/HeuristicPlanningResult');

    % Send feedback to the client
    sendFeedback(planning_server, feedback_msg, default_goal_handle);

    % Return the final result to the client
    result_msg = true;
    setResult(planning_server, result_msg, default_goal_handle);

    result = result_msg;
    success = true;
end