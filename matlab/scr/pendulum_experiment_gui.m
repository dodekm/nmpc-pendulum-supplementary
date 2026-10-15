function app=pendulum_experiment_gui(options)
%PENDULUM_EXPERIMENT_GUI Optional MATLAB observer/operator for AutoID firmware.
% No serial connection or experiment starts until Connect & Record is clicked.
% The onboard controller remains autonomous; MATLAB has no control heartbeat.
if nargin<1,options=struct();end
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab','gui'),fullfile(root,'matlab','acquisition'));
app=PendulumExperimentApp(root,options);
end
