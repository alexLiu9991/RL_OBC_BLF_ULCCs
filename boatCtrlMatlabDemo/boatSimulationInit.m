setenv('WEBOTS_PROJECT', ''); % your webots project path 
setenv('WEBOTS_CONTROLLER_NAME', 'boatCtrlMatlabDemo');
setenv('WEBOTS_VERSION', '2023a');

cd(getenv("WEBOTS_HOME"));
cd('lib/controller/matlab');
launcher;