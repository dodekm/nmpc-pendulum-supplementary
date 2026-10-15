classdef PendulumExperimentApp < handle
    % Optional PC observer/operator. All feedback/identification stays onboard.
    % One acquisition owner, no background serial callback or control heartbeat.
    properties (SetAccess=private)
        Figure
        Model
        IsRecording=false
        SessionDirectory=''
        ConnectionStatus='Disconnected - onboard operation is independent'
    end
    properties (Access=private)
        Root
        Options
        Context=[]
        Widgets=struct()
        Axes=struct()
        Lines=struct()
        LastRender=-Inf
        LastPacketAt=NaN
        Clock
        ClosePending=false
        StopMessage='Physical user button is the primary emergency stop'
        MainLayout
        LiveLayout
        SummaryPanel
        PortScan=struct('state','unscanned','detected',strings(0,1),'available',strings(0,1))
    end
    methods
        function self=PendulumExperimentApp(root,options)
            defaults=struct('port',"COM4",'rig_id',"rig_A4988",'visible',true, ...
                'duration_seconds',Inf, ...
                'port_list_provider',@serialportlist, ...
                'source_factory',[],'output_directory',fullfile(root,'matlab','scr','results','serial'));
            assert(isstruct(options) && isscalar(options),'adaptive:GuiOption','Invalid GUI options.');
            assert(isempty(setdiff(fieldnames(options),fieldnames(defaults))),'adaptive:GuiOption','Unknown GUI option.');
            for name=fieldnames(options)',defaults.(name{1})=options.(name{1});end
            validateattributes(defaults.visible,{'logical'},{'scalar'});
            validateattributes(defaults.duration_seconds,{'double'},{'real','scalar','positive'});
            assert(isa(defaults.port_list_provider,'function_handle'), ...
                'adaptive:GuiOption','port_list_provider must be a function handle.');
            self.Root=root;self.Options=defaults;self.Model=AdaptiveLiveData;self.Clock=tic;
            self.buildWindow();
        end
        function connect(self)
            if self.IsRecording,return;end
            if ~self.Widgets.Ready.Value
                self.message('Prepare and supervise the rig before connecting. Opening the port may reset some adapters.');return;
            end
            self.Model=AdaptiveLiveData;self.IsRecording=true;self.Context=[];
            % Until on_open supplies a NEW session, never associate this
            % attempt's preview with a previous successful recording.
            self.SessionDirectory='';
            self.Widgets.Session.Value={'Opening a new session...'};
            self.LastPacketAt=NaN;
            self.StopMessage='Physical user button is the primary emergency stop';
            self.ConnectionStatus='Opening serial port and a new recording session...';self.setControls();
            adaptive_serial_options=struct('port',string(self.Widgets.Port.Value), ...
                'rig_id',string(self.Widgets.Rig.Value),'show_stop_window',false,'echo_events',false, ...
                'request_metadata',false,'source_factory',self.Options.source_factory, ...
                'duration_seconds',self.Options.duration_seconds, ...
                'output_directory',self.Options.output_directory, ...
                'on_open',@(context)self.opened(context),'on_bytes',@(bytes,time)self.received(bytes,time), ...
                'on_command',@(entry)self.commandCompleted(entry),'on_close',@(directory)self.ended(directory), ...
                'on_idle',@()self.render(false));
            if ~isempty(self.Options.source_factory),adaptive_serial_options.poll_pause_seconds=0;end
            try
                capture_adaptive_serial(adaptive_serial_options);
            catch exception
                self.Model.addEvent("PC acquisition error: "+string(exception.message));
                self.ConnectionStatus='Acquisition failed - rig may still be operating';
                self.IsRecording=false;self.Context=[];self.setControls();self.render(true);
            end
            if self.ClosePending && isgraphics(self.Figure),delete(self.Figure);end
        end
        function stopRecording(self)
            if ~isempty(self.Context)
                self.Context.stop();self.ConnectionStatus='Finishing recording; no hardware stop command sent';
                self.render(true);
            end
        end
        function emergencyStop(self)
            if self.send('X')
                self.Model.clearStopObservation();
                self.StopMessage='Stop requested - awaiting a NEW firmware event; physical button remains primary';
            end
            self.render(true);
        end
        function accepted=send(self,command)
            accepted=false;
            if isempty(self.Context),self.message('Connect and record first.');return;end
            try
                self.Context.command(command);accepted=true;
            catch exception
                self.message(exception.message);
            end
        end
        function saveNote(self)
            if isempty(self.Context),self.message('Notes require an active recording session.');return;end
            text=strjoin(string(self.Widgets.Note.Value),newline);
            try
                self.Context.note(text);self.Model.addEvent("Operator note queued: "+text);
                self.Widgets.Note.Value={''};self.render(true);
            catch exception
                self.message(exception.message);
            end
        end
        function received(self,bytes,time)
            self.LastPacketAt=toc(self.Clock);
            previousStop=self.Model.StopEventObserved;
            self.Model.consume(bytes,time);
            if self.Model.StopEventObserved
                self.StopMessage='Firmware reports emergency stop; event is not request-ID correlated or shaft-speed feedback';
            elseif previousStop
                self.StopMessage='Previous stop observation invalidated by restart; physical button remains primary';
            end
            self.render(false);
        end
        function render(self,force)
            if ~isgraphics(self.Figure),return;end
            if ~force && toc(self.Clock)-self.LastRender<.2,return;end
            self.LastRender=toc(self.Clock);
            self.Widgets.Connection.Text=self.ConnectionStatus;
            if self.IsRecording && isfinite(self.LastPacketAt) && toc(self.Clock)-self.LastPacketAt>5
                self.Widgets.Connection.Text=sprintf('No received bytes for %.1f s; rig may still be operating. No automatic reset/reconnect.',toc(self.Clock)-self.LastPacketAt);
            end
            self.Widgets.Stop.Text=self.StopMessage;
            phase=self.numeric(self.Model.Telemetry,'phase');
            self.Widgets.Phase.Text="Phase: "+self.phaseName(phase);
            frame=self.Model.CurrentFrame;
            if phase==8 && isfinite(self.numeric(frame,'phase'))
                names={'Swing-up','Upright hold','Swing-down','Downward settling','Cycle complete'};
                stage=self.numeric(frame,'phase');
                if stage>=1 && stage<=numel(names) && mod(stage,1)==0
                    self.Widgets.Phase.Text=string(names{stage});
                end
            end
            phi=self.numeric(frame,'phi_raw');reference=self.numeric(frame,'ref_phi');
            referenceErrorDegrees=atan2(sin(phi-reference),cos(phi-reference))*180/pi;
            self.Widgets.AngleError.Text=self.number(referenceErrorDegrees,'%.2f',' deg');
            self.Widgets.WheelSpeed.Text=self.number(self.numeric(frame,'v_sent'),'%.2f',' rad/s');
            self.Widgets.Cycle.Text=self.number(self.numeric(frame,'cycle'),'%.0f','')+" / "+ ...
                self.number(self.numeric(frame,'t_s'),'%.1f',' s');
            alpha=self.numeric(self.Model.Telemetry,'alpha');beta=self.numeric(self.Model.Telemetry,'beta');
            gamma=self.numeric(self.Model.Telemetry,'gamma');committed=self.numeric(self.Model.Telemetry,'committed');
            status='not committed';if committed==1,status='committed';end
            self.Widgets.Model.Text="Onboard model "+status+"  |  alpha "+self.number(alpha,'%.5g','')+ ...
                "  |  beta "+self.number(beta,'%.5g','')+"  |  gamma "+self.number(gamma,'%.5g','');
            self.Widgets.Quality.Text=sprintf('UNVERIFIED live preview | parse errors %d | sequence gaps %d | replay rows %d', ...
                self.Model.MalformedLines,self.Model.SequenceGaps,self.Model.ReplayRows);
            build=self.Model.Build;
            driver='unknown driver';profile=self.numeric(build,'driver_profile');
            if profile==0,driver='A4988';elseif profile==1,driver='HY-DIV268N';end
            precision='unknown precision';singlePrecision=self.numeric(build,'single_precision');
            if singlePrecision==1,precision='float';elseif singlePrecision==0,precision='double';end
            self.Widgets.Build.Text="Telemetry protocol v"+self.number(self.numeric(build,'version'),'%.0f','')+ ...
                "  |  "+driver+"  |  1/"+self.number(self.numeric(build,'microsteps'),'%.0f','')+ ...
                " step  |  "+precision+"  |  "+self.number(self.numeric(build,'core_clock_hz')/1e6,'%.0f',' MHz');
            fields={'solver_exit','iterations','gradient','cost','innovation','innovation_variance', ...
                'control_fault','missed_samples','discarded_pulses','capture_gates','capture_samples','membership'};
            values=cell(numel(fields),2);
            for index=1:numel(fields),values(index,:)={fields{index},char(self.number(self.numeric(frame,fields{index}),'%.7g',''))};end
            self.Widgets.Diagnostics.Data=values;
            labels={'alpha','beta','gamma','bias','c'};units={'s^-2','s^-1','signed ratio','rad/s^2','rad/s^2'};
            cells=cell(5,3);
            for index=1:5,cells(index,:)={labels{index},char(self.number(self.numeric(self.Model.LatestFit,labels{index}),'%.7g','')),units{index}};end
            self.Widgets.Fit.Data=cells;
            validation=self.Model.Validation;
            self.Widgets.Validation.Text=sprintf('Reported validation attempt %g/gen %g: accepted %g, RMS %.6g rad | latest fit exit %g (7 = precision limited)', ...
                self.numeric(validation,'attempt'),self.numeric(validation,'generation'), ...
                self.numeric(validation,'accepted'),self.numeric(validation,'rms'),self.numeric(self.Model.LatestFit,'exit'));
            control=self.Model.Control;
            if ~isempty(control)
                [time,indices]=adaptive_gap_indices(control(:,1),control(:,3),control(:,4));
                self.update(self.Lines.Raw,time,self.gapped(control(:,5),indices)*180/pi);
                self.update(self.Lines.Ekf,time,self.gapped(control(:,6),indices)*180/pi);
                self.update(self.Lines.Reference,time,self.gapped(control(:,7),indices)*180/pi);
                self.update(self.Lines.Speed,time,self.gapped(control(:,9),indices));
                self.update(self.Lines.Acceleration,time,self.gapped(control(:,10),indices));
                self.update(self.Lines.Solver,time,self.gapped(control(:,11),indices)/1000);
                self.update(self.Lines.Delay,time,self.gapped(control(:,12),indices)/1000);
                if ~isempty(self.Options.source_factory)
                    title(self.Axes.Timing,'SIMULATED timing - not MCU measurements');
                    if all(control(:,11)==0 & control(:,12)==0)
                        self.update(self.Lines.Solver,NaN,NaN);self.update(self.Lines.Delay,NaN,NaN);
                    end
                else
                    title(self.Axes.Timing,'Firmware-reported timing');
                end
                title(self.Axes.Angle,sprintf('Current cycle %g - raw angle / EKF / reference',control(end,2)));
            else
                for name={'Raw','Ekf','Reference','Speed','Acceleration','Solver','Delay'}
                    self.update(self.Lines.(name{1}),NaN,NaN);
                end
            end
            id=self.Model.Identification;
            kinds={'Coarse','Validation','Guided'};kind=self.numeric(self.Model.Record,'kind');name='Unknown';
            if isfinite(kind) && kind>=0 && kind<=2 && mod(kind,1)==0,name=kinds{kind+1};end
            title(self.Axes.IdAngle,sprintf('%s preview - attempt %g, generation %g (UNVERIFIED)',name, ...
                self.numeric(self.Model.Record,'attempt'),self.numeric(self.Model.Record,'generation')));
            if ~isempty(id)
                [time,indices]=adaptive_gap_indices(id(:,1),id(:,1),id(:,2));
                self.update(self.Lines.IdAngle,time,self.gapped(id(:,3),indices)*180/pi);
                [x,y]=adaptive_held_command_segments(id);
                self.update(self.Lines.IdSpeed,x,y);
            else
                self.update(self.Lines.IdAngle,NaN,NaN);self.update(self.Lines.IdSpeed,NaN,NaN);
            end
            if ~isempty(self.Model.Events),self.Widgets.Events.Value=cellstr(self.Model.Events);end
            drawnow limitrate nocallbacks
        end
        function path=savePreview(self)
            if isempty(self.SessionDirectory),self.message('Create a recording session first.');path='';return;end
            preview=struct('kind','unverified_bounded_live_preview', ...
                'full_raw_capture',fullfile(self.SessionDirectory,'raw.txt'), ...
                'transport_crc_verified',false,'mechanical_wheel_speed_measured',false, ...
                'control_fields',{self.Model.PlotFields},'current_cycle',self.Model.Control, ...
                'latest_identification',self.Model.Identification,'latest_record',self.Model.Record, ...
                'identification_fields',{{'index','read_elapsed_s','angle_rad','held_speed_command_rad_s','ack_elapsed_s'}}, ...
                'telemetry',self.Model.Telemetry,'build',self.Model.Build,'events',self.Model.Events);
            uuid=char(java.util.UUID.randomUUID());path=fullfile(self.SessionDirectory,['preview_' uuid '.mat']);
            try
                nmpc_save_mat_no_replace(path,struct('preview',preview));
                self.message("Bounded UNVERIFIED preview saved: "+string(path));
            catch exception
                path='';self.message("Preview save failed; raw recording is separate: "+string(exception.message));
            end
        end
        function close(self)
            if self.IsRecording
                answer=uiconfirm(self.Figure, ...
                    'Closing ends PC recording only. The rig continues autonomously. Use its physical button to stop it.', ...
                    'Close recording dashboard?','Options',{'Stop logging and close','Cancel'},'DefaultOption',2,'CancelOption',2);
                if ~strcmp(answer,'Stop logging and close'),return;end
                self.ClosePending=true;self.stopRecording();
            elseif isgraphics(self.Figure)
                delete(self.Figure);
            end
        end
        function delete(self)
            % Object deletion never sends a hardware stop/reset command.
            if ~isempty(self.Context)
                try
                    self.Context.stop();
                catch
                    % Destruction must continue after recorder cleanup failure.
                end
            end
            if ~isempty(self.Figure) && isgraphics(self.Figure),delete(self.Figure);end
        end
    end
    methods (Access=private)
        function opened(self,context)
            self.Context=context;self.SessionDirectory=context.directory;
            self.ConnectionStatus='Recording - reset BOARD now to capture boot (user button released)';
            self.Widgets.Session.Value=cellstr(string(context.directory));self.setControls();
            context.command('H'); % Diagnostic metadata only; no motor start/reset.
            self.render(true);
        end
        function ended(self,directory)
            self.IsRecording=false;self.Context=[];self.SessionDirectory=directory;
            self.ConnectionStatus='Recording closed - onboard application remains independent';
            self.Widgets.Ready.Value=false;
            self.Model.addEvent("PC recorder closed; data retained: "+string(directory));self.setControls();self.render(true);
        end
        function commandCompleted(self,entry)
            self.Model.addEvent("PC "+string(entry.kind)+": "+string(entry.payload)+" ["+string(entry.result)+"]");
            if strcmp(entry.kind,'command') && strcmp(entry.payload,'X')
                if strcmp(entry.result,'serial_write_returned')
                    self.StopMessage='Serial STOP write returned - awaiting firmware emergency-stop event';
                else
                    self.StopMessage='STOP write failed/unconfirmed - use the PHYSICAL user button immediately';
                end
            end
            self.render(true);
        end
        function setControls(self)
            if ~isgraphics(self.Figure),return;end
            active=~isempty(self.Context);self.Widgets.Connect.Enable=self.enabled(~self.IsRecording);
            self.Widgets.Port.Enable=self.enabled(~self.IsRecording);self.Widgets.Rig.Enable=self.enabled(~self.IsRecording);
            self.Widgets.Ready.Enable=self.enabled(~self.IsRecording);self.Widgets.Refresh.Enable=self.enabled(~self.IsRecording);
            for name={'StopRecording','Emergency','Metadata','History','Streaming','SaveNote'}
                self.Widgets.(name{1}).Enable=self.enabled(active);
            end
        end
        function refreshPorts(self)
            if self.IsRecording,return;end
            try
                % Enumeration ONLY: no opening, auto-selecting a different rig,
                % resetting or reconnecting. Retain manually entered ports too.
                detected=self.portNames(self.Options.port_list_provider('all'));
                available=self.portNames(self.Options.port_list_provider('available'));
                selected=string(self.Widgets.Port.Value);
                listed=unique([available;detected;selected],'stable');
                listed=listed(strlength(listed)>0);
                self.Widgets.Port.Items=cellstr(listed);
                self.Widgets.Port.Value=char(selected);
                self.PortScan=struct('state','ok','detected',detected,'available',available);
                busy=setdiff(detected,available,'stable');
                self.Widgets.Port.Tooltip="Available: "+strjoin(available,', ')+newline+ ...
                    "Detected but unavailable: "+strjoin(busy,', ')+newline+ ...
                    "Last scan only; availability can change. Selection is preserved; no port was opened.";
                self.updatePortStatus();
                self.message("Port scan: "+string(self.Widgets.PortStatus.Text));
            catch exception
                self.PortScan.state='failed';
                self.Widgets.Port.Tooltip='Last scan failed; previous items may be stale. Type a port or refresh again.';
                self.updatePortStatus();
                self.message("Port scan failed: "+string(exception.message));
            end
        end
        function updatePortStatus(self)
            % A selection change uses the last snapshot, never another scan or
            % connection. Availability is advisory, not a reservation or guard.
            selected=string(self.Widgets.Port.Value);
            if strcmp(self.PortScan.state,'failed')
                status="Scan failed; manual entry works. See Events.";
            elseif strcmp(self.PortScan.state,'unscanned')
                status="Refresh to detect ports, or type a port manually.";
            elseif strlength(strip(selected))==0
                status="Select or type a serial port before connecting.";
            elseif isempty(self.PortScan.detected) && isempty(self.PortScan.available)
                status="No ports detected. Check USB or enter manually.";
            elseif ismember(selected,self.PortScan.available)
                status=selected+" available at last scan; "+numel(self.PortScan.available)+" available port(s).";
            elseif ismember(selected,self.PortScan.detected)
                status=selected+" unavailable; close other port owners and refresh.";
            else
                status=selected+" not detected. Check USB or choose a listed port.";
            end
            self.Widgets.PortStatus.Text=status;
        end
        function message(self,text)
            self.Model.addEvent("PC: "+string(text));self.render(true);
        end
        function buildWindow(self)
            visible='off';if self.Options.visible,visible='on';end
            self.Figure=uifigure('Name','Reaction-wheel pendulum - autonomous research dashboard', ...
                'Position',[60 60 1280 820],'Visible',visible,'Color',[.96 .97 .98], ...
                'AutoResizeChildren','off', ...
                'CloseRequestFcn',@(~,~)self.close());
            main=uigridlayout(self.Figure,[3 2]);main.ColumnWidth={320,'1x'};main.RowHeight={98,'1x',42};
            self.MainLayout=main;
            heading=uipanel(main,'BackgroundColor',[.06 .13 .22],'BorderType','none');heading.Layout.Row=1;heading.Layout.Column=[1 2];
            h=uigridlayout(heading,[3 1]);h.RowHeight={28,22,22};h.RowSpacing=2;
            h.Padding=[14 4 14 4];h.BackgroundColor=heading.BackgroundColor;
            self.Widgets.Title=uilabel(h,'Text','AUTONOMOUS PENDULUM | research acquisition','FontSize',19,'FontWeight','bold','FontColor',[1 1 1]);
            self.Widgets.Connection=uilabel(h,'Text',self.ConnectionStatus,'FontColor',[.72 .85 .94]);
            self.Widgets.Stop=uilabel(h,'Text',self.StopMessage,'FontColor',[1 .78 .40]);
            panel=uipanel(main,'Title','Connection and operator actions');panel.Layout.Row=2;panel.Layout.Column=1;
            left=uigridlayout(panel,[13 2]);left.RowHeight={22,64,22,28,28,38,34,42,32,32,32,62,54};left.Scrollable='on';
            uilabel(left,'Text','COM port','Tooltip','250000 baud, 8 data bits, no parity, 1 stop bit.');
            self.Widgets.Refresh=uibutton(left,'Text','Refresh ports','Tag','RefreshPorts', ...
                'ButtonPushedFcn',@(~,~)self.refreshPorts());
            ports=uigridlayout(left,[2 1]);ports.RowHeight={28,32};ports.Padding=[0 0 0 0];ports.RowSpacing=2;self.span(ports,2);
            self.Widgets.Port=uidropdown(ports,'Items',{char(self.Options.port)},'Editable','on', ...
                'Value',char(self.Options.port),'Tag','SerialPort', ...
                'ValueChangedFcn',@(~,~)self.updatePortStatus());
            self.Widgets.PortStatus=uilabel(ports,'Text','Refresh to detect ports, or type the COM port manually.', ...
                'Tag','PortScanStatus','WordWrap','on','FontSize',10,'FontColor',[.35 .4 .45]);
            label=uilabel(left,'Text','Rig annotation - does not configure firmware');self.span(label,3);
            self.Widgets.Rig=uidropdown(left,'Items',{'rig_A4988','rig_HY_DIV268N','rig_other'},'Editable','on','Value',char(self.Options.rig_id));self.span(self.Widgets.Rig,4);
            self.Widgets.Ready=uicheckbox(left,'Text','Ready: supervised, down, wheel stopped','Value',false);self.span(self.Widgets.Ready,5);
            self.Widgets.Connect=uibutton(left,'Text','Connect & record','BackgroundColor',[.08 .46 .38],'FontColor',[1 1 1], ...
                'FontWeight','bold','Interruptible','on','BusyAction','cancel', ...
                'ButtonPushedFcn',@(~,~)self.connect());self.span(self.Widgets.Connect,6);
            self.Widgets.StopRecording=uibutton(left,'Text','Stop recording (rig continues)','ButtonPushedFcn',@(~,~)self.stopRecording());self.span(self.Widgets.StopRecording,7);
            self.Widgets.Emergency=uibutton(left,'Text','REQUEST EMERGENCY STOP  [X]','BackgroundColor',[.76 .12 .16],'FontColor',[1 1 1], ...
                'FontWeight','bold','ButtonPushedFcn',@(~,~)self.emergencyStop());self.span(self.Widgets.Emergency,8);
            self.Widgets.Metadata=uibutton(left,'Text','Metadata [H]','ButtonPushedFcn',@(~,~)self.send('H'));
            self.Widgets.History=uibutton(left,'Text','Enable / replay [P]','ButtonPushedFcn',@(~,~)self.send('P'));
            self.Widgets.Streaming=uibutton(left,'Text','Toggle control streaming [D]','ButtonPushedFcn',@(~,~)self.send('D'));self.span(self.Widgets.Streaming,10);
            help=uibutton(left,'Text','How to restart / re-identify','ButtonPushedFcn',@(~,~)self.message( ...
                'No serial restart exists: original downward rest + physically stopped wheel, then a NEW board-button press/release; board RESET performs fresh identification.'));self.span(help,11);
            self.Widgets.Session=uitextarea(left,'Editable','off','Value',{'Session folder appears here after connection.'});self.span(self.Widgets.Session,12);
            note=uilabel(left,'Text','No PC control heartbeat. Closing MATLAB does not disable the rig.','WordWrap','on','FontColor',[.35 .4 .45]);self.span(note,13);
            tabs=uitabgroup(main);tabs.Layout.Row=2;tabs.Layout.Column=2;
            live=uitab(tabs,'Title','Live control');grid=uigridlayout(live,[4 2]);grid.RowHeight={58,26,'1x','1x'};
            grid.Scrollable='on';self.LiveLayout=grid;
            self.SummaryPanel=uipanel(grid,'BorderType','none','BackgroundColor',[.94 .96 .98]);
            self.SummaryPanel.Layout.Row=1;self.SummaryPanel.Layout.Column=[1 2];
            summary=uigridlayout(self.SummaryPanel,[2 4]);summary.RowHeight={16,'1x'};summary.RowSpacing=1;summary.Padding=[10 3 10 3];
            summary.BackgroundColor=self.SummaryPanel.BackgroundColor;
            keys={'Phase','AngleError','WheelSpeed','Cycle'};
            captions={'ONBOARD PHASE','REFERENCE ERROR','WHEEL COMMAND','CYCLE / ELAPSED'};
            for index=1:4
                label=uilabel(summary,'Text',captions{index},'FontSize',10,'FontColor',[.35 .4 .46]);
                label.Layout.Row=1;label.Layout.Column=index;
                value=uilabel(summary,'Text',char(8212),'FontSize',17,'FontWeight','bold','FontColor',[.06 .2 .31]);
                value.Tag=keys{index};value.Layout.Row=2;value.Layout.Column=index;
                self.Widgets.(keys{index})=value;
            end
            self.Widgets.AngleError.Tooltip='Wrapped raw encoder minus commanded angular reference; not a stability certificate.';
            self.Widgets.Phase.FontSize=14;self.Widgets.Phase.WordWrap='on';
            self.Widgets.WheelSpeed.Tooltip='Acknowledged prescribed step rate, not measured wheel-shaft speed.';
            self.Widgets.Model=uilabel(grid,'Text','Onboard model: waiting for firmware','WordWrap','on','FontColor',[.35 .4 .46]);
            self.Widgets.Model.Layout.Row=2;self.Widgets.Model.Layout.Column=[1 2];
            self.Axes.Angle=self.plotAxes(grid,3,1,'Angle (deg)','Cycle time (s)');
            self.Axes.Speed=self.plotAxes(grid,3,2,'Speed command (rad/s)','Cycle time (s)');
            title(self.Axes.Speed,'Prescribed wheel speed - not shaft feedback');
            self.Axes.Acceleration=self.plotAxes(grid,4,1,'Acceleration command (rad/s^2)','Cycle time (s)');
            self.Axes.Timing=self.plotAxes(grid,4,2,'Duration (ms)','Cycle time (s)');
            self.Lines.Raw=plot(self.Axes.Angle,NaN,NaN,'Color',[.02 .36 .60],'LineWidth',1.2,'DisplayName','Encoder');hold(self.Axes.Angle,'on');
            self.Lines.Ekf=plot(self.Axes.Angle,NaN,NaN,'Color',[.90 .42 .12],'LineWidth',1,'DisplayName','EKF');
            self.Lines.Reference=plot(self.Axes.Angle,NaN,NaN,'--','Color',[.35 .40 .46],'DisplayName','Reference');legend(self.Axes.Angle,'Location','best');
            self.Lines.Speed=plot(self.Axes.Speed,NaN,NaN,'Color',[.02 .36 .60],'LineWidth',1.2);
            self.Lines.Acceleration=plot(self.Axes.Acceleration,NaN,NaN,'Color',[.90 .42 .12],'LineWidth',1.2);
            self.Lines.Solver=plot(self.Axes.Timing,NaN,NaN,'Color',[.02 .36 .60],'DisplayName','EKF + preview + NMPC');hold(self.Axes.Timing,'on');
            self.Lines.Delay=plot(self.Axes.Timing,NaN,NaN,'Color',[.90 .42 .12],'DisplayName','Read to command');legend(self.Axes.Timing,'Location','best');
            identification=uitab(tabs,'Title','Identification');id=uigridlayout(identification,[4 1]);id.RowHeight={'1x','1x',145,40};
            self.Axes.IdAngle=self.plotAxes(id,1,1,'Raw encoder angle (deg)','Acquisition time (s)');
            self.Axes.IdSpeed=self.plotAxes(id,2,1,{'Known held command (rad/s)','publication to read'},'Acquisition time (s)');
            self.Axes.IdSpeed.Tag='IdentificationCommandAxes';
            self.Lines.IdAngle=plot(self.Axes.IdAngle,NaN,NaN,'Color',[.02 .36 .60],'LineWidth',1.2);
            self.Lines.IdSpeed=stairs(self.Axes.IdSpeed,NaN,NaN,'Color',[.90 .42 .12],'LineWidth',1.2);
            self.Widgets.Fit=uitable(id,'ColumnName',{'Latest fit (may not be committed)','Value','Units'},'ColumnWidth',{'auto',110,110},'RowName',{});
            self.Widgets.Validation=uilabel(id,'Text','Validation: waiting','WordWrap','on');
            diagnostic=uitab(tabs,'Title','Solver / EKF');dg=uigridlayout(diagnostic,[2 1]);dg.RowHeight={60,'1x'};
            uilabel(dg,'Text','Latest firmware fields. Exit codes are not stability proofs; innovation is pre-correction. Missing older fields remain NaN.','WordWrap','on');
            self.Widgets.Diagnostics=uitable(dg,'ColumnName',{'Field','Latest value'},'ColumnWidth',{'auto',180},'RowName',{});
            events=uitab(tabs,'Title','Events & research notes');eg=uigridlayout(events,[4 1]);eg.RowHeight={36,'1x',85,34};
            self.Widgets.Build=uilabel(eg,'Text','Firmware identity: not yet received','WordWrap','on','Tag','FirmwareIdentity');
            self.Widgets.Events=uitextarea(eg,'Editable','off','Value',{'All raw data are saved independently of this bounded live preview.'});
            self.Widgets.Note=uitextarea(eg,'Value',{'Record observations: missed steps, touches, vibration, heating, experiment outcome...'});
            self.Widgets.SaveNote=uibutton(eg,'Text','Save timestamped operator note','ButtonPushedFcn',@(~,~)self.saveNote());
            files=uitab(tabs,'Title','Data & files');fg=uigridlayout(files,[4 1]);fg.RowHeight={100,36,36,'1x'};
            uilabel(fg,'Text', ['Every received byte is in raw.txt. receive.jsonl holds receipt-chunk timing; session.json and session.mat hold provenance. ' ...
                'actions.jsonl records notes and explicit commands. Full offline CRC/timing validation is separate; the live preview is bounded and unverified.'], ...
                'WordWrap','on');
            uibutton(fg,'Text','Save current bounded preview (.mat)','ButtonPushedFcn',@(~,~)self.savePreview());
            uibutton(fg,'Text','Show session folder in event log','ButtonPushedFcn',@(~,~)self.message(string(self.SessionDirectory)));
            uilabel(fg,'Text',['For a clean run: connect & record BEFORE board reset, leave the rig untouched, ' ...
                'retain boot through identification and complete control cycles. Stop the rig with its physical button. ' ...
                'Stopping recording/closing MATLAB does not stop the autonomous application. No UARTLink software or PC control heartbeat is required.'],'WordWrap','on');
            self.Widgets.Quality=uilabel(main,'Text','Live preview is unverified. Complete raw data remain available for CRC-checked offline analysis.','WordWrap','on');
            self.Widgets.Quality.Layout.Row=3;self.Widgets.Quality.Layout.Column=[1 2];self.setControls();
            self.Figure.SizeChangedFcn=@(~,~)self.reflow();self.reflow();
        end
        function reflow(self)
            % Explicit plot stacking prevents narrow viewports clipping charts.
            width=self.Figure.Position(3);
            sidebar=320;if width<1200,sidebar=290;end
            self.MainLayout.ColumnWidth={sidebar,'1x'};
            names={'Angle','Speed','Acceleration','Timing'};
            if width<1150
                self.LiveLayout.ColumnWidth={'1x'};
                self.LiveLayout.RowHeight={58,40,230,230,230,230};
                self.SummaryPanel.Layout.Column=1;self.Widgets.Model.Layout.Column=1;
                for index=1:4
                    self.Axes.(names{index}).Layout.Row=index+2;
                    self.Axes.(names{index}).Layout.Column=1;
                end
            else
                self.LiveLayout.ColumnWidth={'1x','1x'};
                self.LiveLayout.RowHeight={58,26,'1x','1x'};
                self.SummaryPanel.Layout.Column=[1 2];self.Widgets.Model.Layout.Column=[1 2];
                for index=1:4
                    self.Axes.(names{index}).Layout.Row=3+floor((index-1)/2);
                    self.Axes.(names{index}).Layout.Column=1+mod(index-1,2);
                end
            end
        end
    end
    methods (Static,Access=private)
        function span(widget,row),widget.Layout.Row=row;widget.Layout.Column=[1 2];end
        function axes=plotAxes(parent,row,column,y,x)
            axes=uiaxes(parent,'Box','off','FontSize',10,'XGrid','on','YGrid','on','GridAlpha',.12);
            axes.Layout.Row=row;axes.Layout.Column=column;ylabel(axes,y);xlabel(axes,x);
        end
        function value=numeric(fields,name)
            if strcmp(name,'c') && ~isfield(fields,'c'),name='coulomb';end
            value=NaN;if isfield(fields,name) && isnumeric(fields.(name)) && isscalar(fields.(name)),value=fields.(name);end
        end
        function name=phaseName(index)
            names={'Startup / zero','Coarse excitation','Fitting','Validation settling','Validation acquisition', ...
                'Design preparation','Upright design','Downward design','NMPC full cycle','Cooldown', ...
                'Identification rejected','Manual return','Emergency stop','Hardware fault','Retry braking', ...
                'Controlled initial settling','Nominal numerical preflight','Guided settling','Guided acquisition','Validation assessment'};
            name="unknown";if isfinite(index) && index>=0 && index<numel(names) && mod(index,1)==0,name=string(names{index+1});end
        end
        function value=enabled(condition),value='off';if condition,value='on';end,end
        function text=number(value,format,suffix)
            text=string(char(8212));if isfinite(value),text=string(sprintf(format,value))+suffix;end
        end
        function names=portNames(value)
            if ~(isstring(value) || ischar(value) || iscellstr(value))
                error('adaptive:PortList','Invalid port enumeration result.');
            end
            names=string(value);names=names(:);
            if any(ismissing(names)) || any(contains(names,newline)) || ...
                    any(contains(names,char(13))) || any(contains(names,char(0)))
                error('adaptive:PortList','Invalid port name returned by enumeration.');
            end
            names=strip(names);names=unique(names(strlength(names)>0),'stable');
        end
        function update(line,x,y),line.XData=x;line.YData=y;end
        function output=gapped(input,indices)
            output=nan(size(indices));valid=~isnan(indices);output(valid)=input(indices(valid));
        end
    end
end
