cd ../..
packagePathPwd = pwd;
return;

%%%%%%%%%%%%%%%
[~, packagePath] = system(['rospack find ', 'mission_planner']);
% Verificar si se encontró la ruta del paquete
if status == 0
    packagePath = strtrim(packagePath); % Remueve espacios en blanco o saltos de línea
    disp(['Se encontró el paquete: ', packagePath]);
else
    error(['No se ha encontrado el paquete ', packageName, '. Asegúrate de que ROS esté configurado correctamente y que el paquete exista en el workspace.']);
end

% Generar los mensajes personalizados de ROS en MATLAB
try
    disp('Generando mensajes personalizados de ROS para MATLAB...');
    rosgenmsg(packagePath);
catch ME
    error('Error al generar los mensajes personalizados de ROS: %s', ME.message);
end

% Agregar la ruta de los mensajes generados al path de MATLAB
generatedPath = fullfile(packagePath, 'matlab_msg_gen', 'ros', '+mission_planner'); % Ajusta si es necesario
addpath(generatedPath);
savepath; % Guarda el nuevo path para futuras sesiones

disp('Mensajes personalizados de ROS configurados correctamente en MATLAB.');
return;

%%%%%%%%%%%%%%%%%%
% ROS messages initialization script
% Step 1: get the path of the ROS package from the ROS environment
rosPackagePath = getenv('ROS_PACKAGE_PATH');

% Verify if ROS_PACKAGE_PATH has been found
if isempty(rosPackagePath)
    error('The environment variable ROS_PACKAGE_PATH hasn''t been found');
end

% Split the possible different routes in the ros package path
packagePaths = strsplit(rosPackagePath, pathsep);

% Step 2: Search the route of the mission_planner package
packageName = 'mission_planner';
packagePath = '';

% Paths iteration search
for i = 1:length(packagePaths)
    %? Intentar encontrar el paquete usando rospack find (requiere que ROS esté en el sistema PATH)
    [status, cmdout] = system(['rospack find ', packageName]);
    
    % Verify if the package path has been found
    if status == 0
        packagePath = strtrim(cmdout);
        break;
    end
end

% Verify if the package has been found
if isempty(packagePath)
    error([packageName, ' package hasn''t been found.']);
end

% Step 3: Generate ROS messages for matlab
try
    disp('Generating ROS message files for Matlab...');
    rosgenmsg(packagePath);
catch ME
    error('Error while generating ROS messages: %s', ME.message);
end

% Step 4: Add the new messages folder to the Matlab's path
generatedPath = fullfile(packagePath, '+mission_planner');
addpath(generatedPath);
savepath;

disp('ROS messages setted up successfuly');
disp('Remember to clone and add git@github.com:multirobot-use/task_planner.git to the path')



Error using ros.internal.Parsing.validateFolderPath
The folder path, /home/baldman/grvc_planning_workspace/src/grvc_planning_core/ros_packages/mission_planner
, is invalid or the folder does not exist.

Error in rosgenmsg (line 115)
    folderPath = ros.internal.Parsing.validateFolderPath(folderPath);
 
>> rosgenmsg(packagePathPwd);
Error using rosgenmsg
Unable to find packages with '.msg' files, or '.srv' files, or '.action' files in /home/baldman/git/mission_planner. Each message package must contain '.msg' files in a directory named 'msg', or '.srv' files in
a directory named 'srv', or '.action' files in a directory named 'action', or a combination of any of these directories.
