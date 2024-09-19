% Go to action folder
cd ../action

% Make a new folder and move dependent actions inside it
mkdir('compile_later');
try
    movefile('TaskResult.action', 'compile_later/TaskResult.action');
catch
end

% Go to the folder above the package's folder (it doesn't need to be workspace's src folder)
cd ../..

% Save current path
packagePath = pwd;

% Try to generate independent ROS messages
try
    disp('Generating custom ROS messages for MATLAB...');
    rosgenmsg(packagePath);
catch EM
    error('Error generating custom ROS messages: %s', EM.message);
end

% Add the custom messages' folder to Matlab's path
addpath([packagePath, '/matlab_msg_gen_ros1/glnxa64/install/m']);
 
% Refresh all message class definitions, which requires clearing the workspace
clear classes
rehash toolboxcache

% Move the dependent action files back to the action folder
cd mission_planner/action
movefile('compile_later/TaskResult.action', 'TaskResult.action');

% Remove auxiliary folder
rmdir('compile_later');

% Go to the folder above the package's folder (it doesn't need to be workspace's src folder)
cd ../..

% Save current path
packagePath = pwd;

% Try to generate independent ROS messages
try
    disp('Generating custom ROS messages for MATLAB...');
    rosgenmsg(packagePath);
catch EM
    error('Error generating custom ROS messages: %s', EM.message);
end

% Add the custom messages' folder to Matlab's path and save it
addpath([packagePath, '/matlab_msg_gen_ros1/glnxa64/install/m']);
savepath;
 
% Refresh all message class definitions, which requires clearing the workspace
clear classes
rehash toolboxcache

% Finishing display
disp('Custom ROS messages for Matlab generated successfully');