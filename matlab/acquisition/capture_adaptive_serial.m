function directory=capture_adaptive_serial(overrides)
%CAPTURE_ADAPTIVE_SERIAL Single native MATLAB serial/disk acquisition owner.
% Function scope guarantees cleanup on normal return, error and Ctrl+C.
% Default operation is passive; GUI hooks may queue explicitly requested X.
% No implicit start, reset, motion, stop, reconnect or control heartbeat.
% Every raw byte is retained separately from bounded, unverified presentation.
if nargin<1,overrides=struct();end
sourceFile=[mfilename('fullpath') '.m'];
root=fileparts(fileparts(fileparts(sourceFile)));
% Native storage helpers only; no legacy communication setup or MEX path.
addpath(fullfile(root,'matlab','misc'));
options=struct('port',"COM4",'rig_id',"rig_A4988", ...
    'duration_seconds',Inf,'show_stop_window',true,'echo_events',true, ...
    'request_metadata',false,'output_directory',fullfile(root,'matlab','scr','results','serial'), ...
    'source_factory',[],'poll_pause_seconds',.01,'on_open',[],'on_bytes',[], ...
    'on_close',[],'on_command',[],'on_idle',[]);
options=mergeSettings(options,overrides);
directory=collect(options,sourceFile);
end

function options=mergeSettings(options,overrides)
validateattributes(overrides,{'struct'},{'scalar'});
allowed=fieldnames(options);
unknown=setdiff(fieldnames(overrides),allowed);
assert(isempty(unknown),'adaptive:RecorderOption','Unknown recorder option.');
for name=fieldnames(overrides)',options.(name{1})=overrides.(name{1});end
end

function directory=collect(options,sourceFile)
root=fileparts(fileparts(fileparts(sourceFile)));
for name={'on_open','on_bytes','on_close','on_command','on_idle'}
    assert(isempty(options.(name{1})) || isa(options.(name{1}),'function_handle'), ...
        'adaptive:RecorderOption','Invalid presentation callback.');
end
validateattributes(options.port,{'char','string'},{'scalartext','nonempty'});
validateattributes(options.rig_id,{'char','string'},{'scalartext','nonempty'});
validateattributes(options.output_directory,{'char','string'},{'scalartext','nonempty'});
% A scalar string "" is a nonempty MATLAB array, but not a usable identifier.
for name={'port','rig_id','output_directory'}
    assert(~ismissing(string(options.(name{1}))) && ...
        strlength(strtrim(string(options.(name{1}))))>0, ...
        'adaptive:RecorderOption','%s must contain nonblank text.',name{1});
end
validateattributes(options.duration_seconds,{'double'},{'real','scalar','positive'});
assert(~isnan(options.duration_seconds),'adaptive:RecorderOption','Invalid duration.');
validateattributes(options.poll_pause_seconds,{'double'},{'real','finite','scalar','nonnegative','<=',.1});
for name={'show_stop_window','echo_events','request_metadata'}
    validateattributes(options.(name{1}),{'logical'},{'scalar'});
end
assert(isempty(options.source_factory) || isa(options.source_factory,'function_handle'), ...
    'adaptive:RecorderOption','source_factory must be empty or a test factory.');
simulated=~isempty(options.source_factory);
assert(simulated || options.poll_pause_seconds>0,'adaptive:RecorderOption','Live acquisition requires yielding to MATLAB.');
parent=nmpc_canonical_path(options.output_directory);
canonicalRoot=nmpc_canonical_path(root);
assert(startsWith(parent,[canonicalRoot filesep],'IgnoreCase',ispc), ...
    'adaptive:OutputPath','Capture output must stay inside this project.');
uuid=char(java.util.UUID.randomUUID());
stamp=char(datetime('now','TimeZone','UTC','Format','yyyyMMdd''T''HHmmss''Z'''));
directory=adaptive_create_session_directory(parent,['adaptive_serial_' stamp '_' uuid]);

state=struct('options',options,'directory',directory,'source_file',sourceFile, ...
    'raw_file',fullfile(directory,'raw.txt'),'receive_file',fullfile(directory,'receive.jsonl'), ...
    'raw_fid',-1,'receive_fid',-1,'actions_fid',-1,'source',[],'window',[], ...
    'clock',tic,'started_utc',utcNow(),'bytes',0,'chunks',0,'last_tick',0, ...
    'pending',[],'stop_requested',false,'status','interrupted', ...
    'error_identifier','','error_message','','finalization_error','','finalization_errors',{{}}, ...
    'source_hashes',sourceHashes(sourceFile,root), ...
    'requests',struct([]),'simulated',simulated,'finalized',false,'echo_overflows',0, ...
    'queue',{{}},'emergency_pending',false,'emergency_tick',0,'control_commands_sent',false, ...
    'control_commands_requested',false, ...
    'view_errors',0,'queue_rejections',0);
% A shared HANDLE holds changing resource IDs/transaction state. The cleanup
% function is not nested and remains valid during error/Ctrl+C function unwind.
holder=containers.Map({'state'},{state});
cleanup=onCleanup(@()finalizeCapture(holder)); %#ok<NASGU>
tail='';discardEcho=false;lastProgress=0;
try
    state.raw_fid=openBinary(state.raw_file);holder('state')=state;
    state.receive_fid=openBinary(state.receive_file);holder('state')=state;
    state.actions_fid=openBinary(fullfile(directory,'actions.jsonl'));holder('state')=state;
    if simulated
        source=options.source_factory(options.port,250000);
    else
        source=serialport(options.port,250000,'Timeout',.25, ...
            'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');
    end
    % A factory contract failure must still close any handles it created.
    if isa(source,'handle'),state.source=source;holder('state')=state;end
    assert(isa(source,'handle') && isscalar(source),'adaptive:SourceContract','Invalid serial source.');
    setDTR(source,false);setRTS(source,false);
    if options.request_metadata
        write(source,uint8('h'),'uint8'); % Metadata only, NOT a motor command.
        state.requests=struct('command','h','host_elapsed_ticks',receiptTicks(state.clock));
        holder('state')=state;
    end
    if options.show_stop_window && usejava('desktop')
        state.window=figure('Name','Serial recording - rig continues independently', ...
            'NumberTitle','off','MenuBar','none','ToolBar','none','Position',[120 120 510 130], ...
            'CloseRequestFcn',@(figureHandle,~)requestStop(holder,figureHandle));
        holder('state')=state;
        uicontrol(state.window,'Style','text','Position',[15 76 480 36], ...
            'String','This window does NOT stop the rig. Use its physical user button.', ...
            'FontSize',10);
        uicontrol(state.window,'Style','pushbutton','Position',[35 25 440 38], ...
            'String','Stop recording ONLY (rig continues)', ...
            'Callback',@(~,~)requestStop(holder,[]),'FontSize',11);
    end
    kind='Listening';if simulated,kind='SIMULATED source - no hardware opened';end
    fprintf('%s [%s] at 250000 baud. Session:\n%s\n',kind,options.port,directory);
    fprintf('Record BEFORE reset. Physical user button stops rig; recorder button/Ctrl+C stops logging only.\n');
    context=struct('directory',directory,'stop',@()requestStop(holder,[]), ...
        'command',@(command)queueAction(holder,'command',command), ...
        'note',@(text)queueAction(holder,'operator_note',text));
    viewCallback(holder,'on_open',context);
    while true
        for actionBudget=1:17
            if ~serviceActions(holder),break;end % Priority STOP + up to 16 queued actions.
        end
        state=holder('state');
        if state.stop_requested,state.status='recording_stopped';break;end
        if toc(state.clock)>=options.duration_seconds,state.status='recording_duration_elapsed';break;end
        available=source.NumBytesAvailable;
        validateattributes(available,{'double'},{'real','finite','scalar','integer','nonnegative'});
        if available>0
            count=min(8192,available);bytes=read(source,count,'uint8');
            validateattributes(bytes,{'numeric'},{'real','finite','vector','integer','>=',0,'<=',255});
            assert(~isempty(bytes) && numel(bytes)<=count,'adaptive:SourceContract','Invalid returned chunk size.');
            state=appendChunk(holder,uint8(bytes(:)),receiptTicks(state.clock));
            assert(numel(bytes)==count,'adaptive:ShortRead', ...
                'Incomplete serial read; every returned byte was saved before stopping.');
            viewCallback(holder,'on_bytes',uint8(bytes(:)),toc(state.clock));
            state=holder('state');
            if options.echo_events
                [tail,discardEcho,overflows]=echoLines(tail,discardEcho,bytes);
                state.echo_overflows=state.echo_overflows+overflows;holder('state')=state;
            end
        elseif simulated && source.EndOfInput
            state.status='simulation_eof';break;
        else
            pause(options.poll_pause_seconds);
        end
        if toc(state.clock)-lastProgress>=5
            fprintf('Recording: %.1f s, %.0f bytes. Control remains onboard.\n',toc(state.clock),state.bytes);
            lastProgress=toc(state.clock);
        end
        % UI callbacks can only request a stop; a single owner reads/writes data.
        if available>0 && options.poll_pause_seconds>0,pause(options.poll_pause_seconds);end
        viewCallback(holder,'on_idle');
    end
    holder('state')=state;
    finalizeCapture(holder);
    final=holder('state');
    assert(isempty(final.finalization_error),'adaptive:Finalize','%s',final.finalization_error);
catch exception
    state=holder('state');
    if ~state.finalized
        state.status='recording_error';
        state.error_identifier=exception.identifier;state.error_message=exception.message;
        holder('state')=state;finalizeCapture(holder);
    end
    rethrow(exception);
end
end

function queueAction(holder,kind,payload)
state=holder('state');
assert(~state.finalized && ~isempty(state.source),'adaptive:NotConnected','The recorder connection is closed.');
payload=char(string(payload));
if strcmp(kind,'command')
    assert(isscalar(payload) && ismember(upper(payload),'HDPX'),'adaptive:Command','Unsupported firmware command.');
    payload=upper(payload);
    if payload=='X'
        state.emergency_pending=true;state.emergency_tick=receiptTicks(state.clock);
        state.control_commands_requested=true; % Includes interruption before transmission.
        holder('state')=state;return;
    end
else
    assert(~isempty(strtrim(payload)) && numel(payload)<=2000,'adaptive:Note','Provide a nonempty note of at most 2000 characters.');
end
if numel(state.queue)>=16
    state.queue_rejections=state.queue_rejections+1;holder('state')=state;
    error('adaptive:ActionQueue','Too many pending non-emergency actions.');
end
state.queue{end+1}=struct('kind',kind,'payload',payload,'requested_tick',receiptTicks(state.clock));
holder('state')=state;
end
function processed=serviceActions(holder)
processed=false;
state=holder('state');
if state.emergency_pending
    action=struct('kind','command','payload','X','requested_tick',state.emergency_tick);
    state.emergency_pending=false;
elseif ~isempty(state.queue)
    action=state.queue{1};state.queue(1)=[];
else
    return;
end
holder('state')=state;
processed=true;
entry=struct('kind',action.kind,'payload',action.payload,'requested_host_elapsed_ticks',action.requested_tick, ...
    'processed_host_elapsed_ticks',receiptTicks(state.clock),'utc',utcNow(), ...
    'result','recorded','firmware_acknowledged_at_send',false);
try
    if strcmp(action.kind,'command')
        if action.payload=='X',state.control_commands_requested=true;holder('state')=state;end
        write(state.source,uint8(action.payload),'uint8');
        entry.result='serial_write_returned';
        if action.payload=='X',state.control_commands_sent=true;end
        if action.payload=='H',state.options.request_metadata=true;end
    end
catch exception
    entry.result='serial_write_failed';entry.error_identifier=exception.identifier;
    entry.error_message=exception.message;writeAction(state.actions_fid,entry);
    viewCallback(holder,'on_command',entry);rethrow(exception);
end
holder('state')=state;writeAction(state.actions_fid,entry);
viewCallback(holder,'on_command',entry);
end
function writeAction(id,entry)
text=unicode2native([jsonencode(entry) newline],'UTF-8');
assert(fwrite(id,text,'uint8')==numel(text),'adaptive:ActionWrite','Cannot record operator action.');
end
function viewCallback(holder,name,varargin)
state=holder('state');callback=state.options.(name);
if isempty(callback),return;end
try,callback(varargin{:});
catch exception
    state=holder('state');state.view_errors=state.view_errors+1;
    state.options.(name)=[];holder('state')=state; % Broken presentation must not flood/slow acquisition.
    warning('adaptive:ViewCallback','Presentation failed; raw acquisition continues: %s',exception.message);
end
end

function id=openBinary(path)
[id,message]=fopen(path,'wb');assert(id>=0,'adaptive:CaptureOpen','%s',message);
end
function tick=receiptTicks(clock)
tick=round(toc(clock)*1e6); % Monotonic host RECEIPT microseconds, not MCU time.
assert(isfinite(tick) && tick>=0 && tick<=flintmax,'adaptive:Clock','Receipt counter is not representable.');
end
function text=utcNow()
text=char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ss.SSS''Z'''));
end
function state=appendChunk(holder,bytes,tick)
state=holder('state');tick=max(tick,state.last_tick);
% Store the transaction BEFORE writing. Cleanup can complete a partially
% written receive-index row using its original receipt time, without guessing.
state.pending=struct('offset',state.bytes,'bytes',numel(bytes), ...
    'index_start',ftell(state.receive_fid),'tick',tick);holder('state')=state;
written=fwrite(state.raw_fid,bytes,'uint8');
assert(written==numel(bytes),'adaptive:CaptureWrite','Incomplete raw file write.');
writeIndex(state.receive_fid,state.pending.offset,written,tick);
state.bytes=state.bytes+written;state.chunks=state.chunks+1;state.last_tick=tick;
state.pending=[];holder('state')=state;
end
function writeIndex(id,offset,count,tick)
% Explicit decimal integers: JSON exponential doubles would violate the
% downstream receive-index integer contract despite representing whole values.
row=sprintf('{"offset":%.0f,"bytes":%d,"host_elapsed_ticks":%.0f}\n',offset,count,tick);
assert(fwrite(id,uint8(row),'uint8')==numel(row),'adaptive:IndexWrite','Incomplete receive-index write.');
end
function requestStop(holder,window)
state=holder('state');state.stop_requested=true;holder('state')=state;
if ~isempty(window) && isgraphics(window),delete(window);end
end
function [tail,discard,overflows]=echoLines(tail,discard,bytes)
text=[tail char(bytes(:)')];tail='';overflows=0;
while ~isempty(text)
    newline=find(text==char(10),1);
    if isempty(newline)
        if numel(text)>8192,discard=true;overflows=overflows+1;
        elseif ~discard,tail=text;end
        return;
    end
    line=strrep(text(1:newline-1),char(13),'');text=text(newline+1:end);
    if ~discard && numel(line)<=8192 && startsWith(line, ...
            {'event,','adaptive,','id_fit,','id_validation,','id_design,','id_refinement,'})
        fprintf('%s\n',line);
    end
    discard=false;
end
end
function finalizeCapture(holder)
state=holder('state');if state.finalized,return;end
state.finalized=true;holder('state')=state;
% Never send x, reset, speed or any other command on closing the PC recorder.
if ~isempty(state.source)
    try,delete(state.source);catch exception,state=finalizationFailure(state,'serial_close',exception);end
end
if ~isempty(state.window) && isgraphics(state.window),delete(state.window);end
try
    if ~isempty(state.pending) && state.raw_fid>=0 && state.receive_fid>=0
        assert(fseek(state.raw_fid,0,'eof')==0,'adaptive:Repair','Cannot inspect pending raw write.');
        actual=ftell(state.raw_fid)-state.pending.offset;
        assert(actual>=0 && actual<=state.pending.bytes,'adaptive:Repair','Invalid pending raw extent.');
        if actual>0
            assert(fseek(state.receive_fid,state.pending.index_start,'bof')==0, ...
                'adaptive:Repair','Cannot complete pending receive index.');
            writeIndex(state.receive_fid,state.pending.offset,actual,state.pending.tick);
            state.bytes=state.pending.offset+actual;state.chunks=state.chunks+1;
        end
    end
catch exception
    state=finalizationFailure(state,'receive_transaction',exception);
end
for name={'raw_fid','receive_fid','actions_fid'}
    if state.(name{1})>=0
        try
            assert(fclose(state.(name{1}))==0,'adaptive:CaptureClose','File close failed.');
        catch exception,state=finalizationFailure(state,['close_' name{1}],exception);end
        state.(name{1})=-1;
    end
end
try
    root=fileparts(fileparts(fileparts(state.source_file)));
    afterSources=sourceHashes(state.source_file,root);
    sourcesUnchanged=isequal(state.source_hashes,afterSources);
    if ~sourcesUnchanged
        state=finalizationFailure(state,'source_correspondence', ...
            MException('adaptive:SourceChanged','Recorder/storage sources changed during capture.'));
    end
    mode='Device';observation='hardware';if state.simulated,mode='Mock';observation='simulation';end
    manifest=struct('schema',1,'recorder','MATLAB','matlab_version',version, ...
        'started_utc',state.started_utc,'ended_utc',utcNow(),'rig_id',char(state.options.rig_id), ...
        'status',state.status,'error',state.error_message,'error_identifier',state.error_identifier, ...
        'finalization_error',state.finalization_error,'finalization_errors',{state.finalization_errors}, ...
        'finalization_complete',isempty(state.finalization_error), ...
        'authoritative_manifest','session.json','session_mat_is_not_a_commit_marker',true, ...
        'mode',mode,'observation_class',observation, ...
        'port',char(state.options.port),'baud',250000,'dtr',false,'rts',false, ...
        'bytes',state.bytes,'receive_chunks',state.chunks, ...
        'raw_sha256',nmpc_file_sha256(state.raw_file),'receive_sha256',nmpc_file_sha256(state.receive_file), ...
        'recorder_sha256',state.source_hashes(1).sha256, ...
        'source_sha256',state.source_hashes,'source_sha256_after',afterSources, ...
        'sources_unchanged_during_capture',sourcesUnchanged,'host_clock_frequency_hz',1e6, ...
        'host_timestamps_are_receive_times_not_sensor_acquisition',true, ...
        'metadata_requested',state.options.request_metadata,'read_only_requests',state.requests, ...
        'identification_readback_requested',false,'control_commands_sent',state.control_commands_sent, ...
        'control_commands_requested',state.control_commands_requested, ...
        'automatic_reconnect_or_reset',false,'replay_timestamps_are_synthetic',false, ...
        'actions_sha256',nmpc_file_sha256(fullfile(state.directory,'actions.jsonl')), ...
        'presentation_errors',state.view_errors,'action_queue_rejections',state.queue_rejections, ...
        'unprocessed_actions',numel(state.queue),'emergency_request_unsent',state.emergency_pending, ...
        'console_overflow_does_not_modify_raw',true,'console_overflows',state.echo_overflows);
    % Publish the convenience MAT first. JSON is the authoritative final commit:
    % any MAT failure must be visible there, never hidden by an earlier JSON save.
    manifest.session_mat_published=true;
    session=struct('directory',state.directory,'raw_file',state.raw_file, ...
        'receive_file',state.receive_file,'manifest',manifest);
    try
        nmpc_save_mat_no_replace(fullfile(state.directory,'session.mat'),struct('session',session));
    catch exception
        state=finalizationFailure(state,'session_mat',exception);
        manifest.session_mat_published=false;
        manifest.finalization_error=state.finalization_error;
        manifest.finalization_errors=state.finalization_errors;
        manifest.finalization_complete=false;
    end
    nmpc_write_text_no_replace(fullfile(state.directory,'session.json'), ...
        [jsonencode(manifest,'PrettyPrint',true) newline]);
    fprintf('Retained %.0f raw bytes: %s\nCapture status: %s. Closing logging sends no hardware command.\n', ...
        state.bytes,state.directory,state.status);
    if state.control_commands_requested
        fprintf('An EXPLICIT hardware stop was requested; inspect firmware events for confirmation.\n');
    end
catch exception
    state=finalizationFailure(state,'manifest_publication',exception);
end
holder('state')=state;
viewCallback(holder,'on_close',state.directory);
if ~isempty(state.finalization_error)
    warning('adaptive:Finalize','Raw files retained; finalization incomplete: %s',state.finalization_error);
end
end
function state=finalizationFailure(state,stage,exception)
state.finalization_errors{end+1}=struct('stage',stage,'identifier',exception.identifier, ...
    'message',exception.message);
messages=cellfun(@(item)[item.stage ': ' item.message],state.finalization_errors,'UniformOutput',false);
state.finalization_error=strjoin(messages,newline);
end
function hashes=sourceHashes(sourceFile,root)
files={sourceFile,fullfile(root,'matlab','acquisition','adaptive_create_session_directory.m'), ...
    fullfile(root,'matlab','misc','nmpc_save_mat_no_replace.m'), ...
    fullfile(root,'matlab','misc','nmpc_save_mat_record.m'), ...
    fullfile(root,'matlab','misc','nmpc_publish_file_no_replace.m'), ...
    fullfile(root,'matlab','misc','nmpc_publish_record.m'), ...
    fullfile(root,'matlab','misc','nmpc_canonical_path.m'), ...
    fullfile(root,'matlab','misc','nmpc_write_text_no_replace.m'), ...
    fullfile(root,'matlab','misc','nmpc_write_text_record.m'), ...
    fullfile(root,'matlab','misc','nmpc_file_sha256.m')};
hashes=struct('file',{},'sha256',{});
for index=1:numel(files)
    hashes(index)=struct('file',strrep(files{index}(numel(root)+2:end),'\','/'), ...
        'sha256',nmpc_file_sha256(files{index}));
end
end
