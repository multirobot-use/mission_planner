%% Initialize ROS-Matlab connection
% Define ROS master's IP address and port
ip_address = 'http://localhost:11311';

% Check if ROS master's connection is alive
if not(ros.internal.Global.isNodeActive)
    % Initialize ROS master's connection
    rosinit(ip_address);
end

% Read the mission id from the config file
config_file_path = '../config/conf.yaml';
config_file = fopen(config_file_path, 'r');
scenario_id = fscanf(config_file, 'mission_id: %s');

% Load scenario's data
[Agent, Task] = scenario(scenario_id);
userData.Agent = Agent;
userData.Task = Task;

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
    ExecuteGoalFcn={@heuristicPlanningCallback, userData}, DataFormat="struct");    

disp('Heuristic planning action server is ready');

% TODO: Create action server to answer repair requests

% Initialize loop variables
ros_ok = true;
mission_over = false;

% Run while ros::ok() or mission not over
while ros_ok && not(mission_over)
    % Check if ros is still okay
    try
        [~] = evalc('rosnode list');
    catch
        ros_ok = false;
    end

    % Check if mission is over
    % Receive a message from mission over subscriber, waiting up to 0.1s
    % mission_over = receive(mission_over_sub, 0.1);
    if not(isempty(mission_over_sub.LatestMessage)) && mission_over_sub.LatestMessage.Value == true
        mission_over = true;
    end

    pause(0.5);
end

%% End connection with ROS
rosshutdown;
clear all;

function [result, success] = heuristicPlanningCallback(src, goal_msg, feedback_msg, result_msg, userData)
    % Inputs:
    % src is the associated action server object
    % goal_msg is the goal message sent by the action client
    % feedback_msg is the response message (feedback)
    % result_msg is the result message
    % Outputs:
    % result is an action result message
    % success is a bool variable, true if the goal was reached, false if the goal was aborted or preempted

    disp('Received a new planning request')

    % Extract from userData the available robots
    robot_counter = 1;
    for robot = 1:size(userData.Agent, 2)
        if ismember(userData.Agent(robot).name, goal_msg.AvailableAgents)
            available_agents(robot_counter) = userData.Agent(robot);
            robot_counter = robot_counter + 1;
        end
    end

    % Add the recharge task to the pending tasks structure
    pending_tasks(1) = userData.Task(1);
    % Extract from userData the pending tasks
    task_counter = 2;
    for task = 2:1:size(userData.Task, 2)
        if ismember(userData.Task(task).name, goal_msg.RemainingTasks)
            pending_tasks(task_counter) = userData.Task(task);
            task_counter = task_counter + 1;
        end
    end

    % Send feedback to the client
    feedback_msg.Status = 'Executing heuristic planner...';
    sendFeedback(src, feedback_msg);

    result = rosmessage('mission_planner/HeuristicPlanningResult', 'DataFormat', 'struct');

    % Call the heuristic task allocator
    try
        [Agent, Task, ~] = heuristicTaskAllocator(available_agents, pending_tasks, 4, 9);
        result.Success = true;
    catch
        result.Success = false;
    end

    % Extract the results from Agent structure and put them in result_msg
    if result.Success == true
        for robot = 1:size(Agent, 2)
            result.PlanningResult(robot) = rosmessage('mission_planner/TaskQueue', 'DataFormat', 'struct');
            result.PlanningResult(robot).AgentId = Agent(robot).name;
            for task = 2:size(Agent(robot).queue, 2)
                result.PlanningResult(robot).Queue(task - 1) = rosmessage('mission_planner/Task', 'DataFormat', 'struct');
                result.PlanningResult(robot).Queue(task - 1).Id = Task(Agent(robot).queue(task)).name;
            end
        end
    end

    % Send feedback to the client
    feedback_msg.Status = 'Sending back the panning result...';
    sendFeedback(src, feedback_msg);

    success = result.Success;
end