%% Passive standalone-firmware recording -- edit COM/rig, then press Run.
% Prepare supervised original downward rest and a physically stopped wheel.
% RESET the board only AFTER "Listening" appears (user button released).
% The MCU operates autonomously. Closing this logger never stops the rig.
% The physical user button stops the RIG; Stop Recording/Ctrl+C stops logging.
% No UARTLink, PowerShell or Python is used for native acquisition.
captureSettings=struct('port',"COM4",'rig_id',"rig_A4988", ...
    'duration_seconds',Inf,'show_stop_window',true,'echo_events',true, ...
    'request_metadata',false);
% Test/programmatic overrides are validated by the shared acquisition engine.
if exist('adaptive_serial_options','var')
    validateattributes(adaptive_serial_options,{'struct'},{'scalar'});
    for name=fieldnames(adaptive_serial_options)'
        captureSettings.(name{1})=adaptive_serial_options.(name{1});
    end
end
projectRoot=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,'matlab','acquisition'));
adaptive_serial_session=capture_adaptive_serial(captureSettings);
