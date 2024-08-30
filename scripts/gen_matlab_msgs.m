% Define the route to the workspace containing the needed package
folderpath = 'path_to_your_catkin_ws/src';  % Cambia esta ruta a la ubicación de tu paquete
folderpath = fullfile('planning_workspace','src');

% Generar los archivos de mensajes personalizados
rosgenmsg(folderpath);

% Agregar la carpeta generada al path de MATLAB
addpath('path_to_your_catkin_ws/src/install/m');
savepath;

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

%%%%%%%%%%%%%%%%%%
% Iniciar script de inicialización para mensajes de ROS en MATLAB

% Paso 1: Obtener la ruta del ROS_PACKAGE_PATH desde el entorno de ROS
rosPackagePath = getenv('ROS_PACKAGE_PATH');

% Verificar si se ha encontrado el ROS_PACKAGE_PATH
if isempty(rosPackagePath)
    error('No se ha encontrado la variable de entorno ROS_PACKAGE_PATH. Asegúrate de que ROS esté configurado correctamente.');
end

% Separar las posibles múltiples rutas en ROS_PACKAGE_PATH
packagePaths = strsplit(rosPackagePath, pathsep);

% Paso 2: Buscar la ruta del paquete que contiene los mensajes personalizados
% Modifica el nombre del paquete a buscar según tus necesidades
packageName = 'mission_planner'; % Cambia esto al nombre de tu paquete con mensajes personalizados
packagePath = '';

% Iterar sobre las rutas para encontrar la del paquete deseado
for i = 1:length(packagePaths)
    % Intentar encontrar el paquete usando rospack find (requiere que ROS esté en el sistema PATH)
    [status, cmdout] = system(['rospack find ', packageName]);
    
    % Verificar si se encontró la ruta del paquete
    if status == 0
        packagePath = strtrim(cmdout); % Remover espacios en blanco
        break;
    end
end

% Verificar si se encontró el paquete
if isempty(packagePath)
    error(['No se ha encontrado el paquete ', packageName, '. Asegúrate de que el paquete está en tu workspace de ROS y configurado correctamente.']);
end

% Paso 3: Generar los mensajes personalizados
try
    disp('Generando mensajes personalizados de ROS para MATLAB...');
    rosgenmsg(packagePath);
catch ME
    error('Error al generar los mensajes personalizados de ROS: %s', ME.message);
end

% Paso 4: Agregar la carpeta generada al path de MATLAB
generatedPath = fullfile(packagePath, 'matlab_msg_gen', 'ros', '+mission_planner'); % Ajusta según sea necesario
addpath(generatedPath);
savepath; % Guarda el nuevo path para futuras sesiones

disp('Mensajes personalizados de ROS configurados correctamente en MATLAB.');
