classdef AdaptiveLiveData < handle
    % Bounded, UNVERIFIED live preview. Scientific raw acquisition is separate.
    % Never fits models, fills missing data, or claims measured wheel speed.
    properties (SetAccess=private)
        Telemetry=struct()
        Build=struct()
        LatestFit=struct()
        Validation=struct()
        Configuration=struct()
        CurrentFrame=struct()
        Control=zeros(0,20)
        Identification=zeros(0,5)
        Record=struct()
        Events=strings(0,1)
        MalformedLines=0
        ReplayRows=0
        SequenceGaps=0
        StopEventObserved=false
        BootCount=0
        LastReceiptSeconds=NaN
    end
    properties (Constant)
        PlotFields={'seq','cycle','tick','t_s','phi_raw','phi_est','ref_phi','omega_est', ...
            'v_sent','u_sent','solve_us','sample_to_command_us','gradient','membership', ...
            'captured','relative_E_error','innovation','innovation_variance','discarded_pulses','missed_samples'}
    end
    properties (Access=private)
        Tail=''
        DiscardTail=false
        Header={}
        LastSequence=NaN
        LastCycle=NaN
        OriginUs=NaN
    end
    methods
        function consume(self,bytes,receiptSeconds)
            self.LastReceiptSeconds=receiptSeconds;
            text=[self.Tail char(bytes(:)')];self.Tail='';
            while ~isempty(text)
                newline=find(text==char(10),1);
                if isempty(newline)
                    if numel(text)>8192
                        if ~self.DiscardTail,self.MalformedLines=self.MalformedLines+1;end
                        self.DiscardTail=true;
                    elseif ~self.DiscardTail,self.Tail=text;end
                    return;
                end
                line=strrep(text(1:newline-1),char(13),'');text=text(newline+1:end);
                if ~self.DiscardTail
                    if numel(line)>8192
                        self.MalformedLines=self.MalformedLines+1;
                    else
                        self.parseLine(line);
                    end
                end
                self.DiscardTail=false;
            end
        end
        function addEvent(self,text)
            self.Events(end+1,1)=string(text);
            if numel(self.Events)>150,self.Events=self.Events(end-149:end);end
        end
        function value=controlValue(self,name)
            value=NaN;
            if ~isempty(self.Control)
                index=find(strcmp(self.PlotFields,name),1);
                if ~isempty(index),value=self.Control(end,index);end
            end
        end
        function clearStopObservation(self)
            % No protocol request ID exists; an older event is not fresh evidence.
            self.StopEventObserved=false;
        end
    end
    methods (Access=private)
        function parseLine(self,line)
            if isempty(line),return;end
            tokens=cellstr(split(string(line),','));kind=tokens{1};
            try
                switch kind
                    case 'control_diag_header'
                        self.Header={}; % Do not interpret a changed/bad schema using an old header.
                        names=tokens(2:end);
                        if numel(unique(names))~=numel(names) || any(~cellfun(@isvarname,names)) ...
                                || ~all(ismember({'seq','cycle','tick','t_s','phi_raw'},names))
                            error('adaptive:LiveSchema','Invalid control header.');
                        end
                        self.Header=names;
                    case 'control_diag'
                        if isempty(self.Header) || numel(tokens)-1~=numel(self.Header)
                            error('adaptive:LiveFrame','Control header missing or column count changed.');
                        end
                        frame=cell2struct(num2cell(str2double(tokens(2:end))),self.Header,1);
                        required=[frame.seq frame.cycle frame.tick frame.t_s frame.phi_raw];
                        if any(~isfinite(required)) || any(required(1:3)<0) ...
                                || any(required(1:3)>2^32-1) || any(mod(required(1:3),1)~=0)
                            error('adaptive:LiveFrame','Invalid control counters/time/encoder.');
                        end
                        if frame.cycle~=self.LastCycle
                            if isfinite(self.LastCycle) && mod(frame.cycle-self.LastCycle,2^32)>=2^31
                                self.ReplayRows=self.ReplayRows+1;return;
                            end
                            self.Control=zeros(0,20);self.LastSequence=NaN;self.LastCycle=frame.cycle;
                        end
                        if isfinite(self.LastSequence)
                            difference=mod(frame.seq-self.LastSequence,2^32);
                            if difference==0 || difference>=2^31,self.ReplayRows=self.ReplayRows+1;return;end
                            self.SequenceGaps=self.SequenceGaps+max(0,difference-1);
                        end
                        self.LastSequence=frame.seq;row=nan(1,20);
                        self.CurrentFrame=frame;
                        for index=1:numel(self.PlotFields)
                            name=self.PlotFields{index};if isfield(frame,name),row(index)=frame.(name);end
                        end
                        self.Control(end+1,:)=row;
                        if size(self.Control,1)>3600,self.Control=self.Control(end-3599:end,:);end
                    case 'adaptive',self.Telemetry=self.keyValues(tokens(2:end));
                    case 'diagnostics_build',self.Build=self.keyValues(tokens(2:end));
                    case 'control_config',self.Configuration=self.keyValues(tokens(2:end));
                    case 'id_fit',self.LatestFit=self.keyValues(tokens(2:end));self.addEvent(line);
                    case 'id_validation',self.Validation=self.keyValues(tokens(2:end));self.addEvent(line);
                    case {'id_design','id_refinement'},self.addEvent(line);
                    case 'id_archive_begin'
                        self.Record=self.keyValues(tokens(2:end));self.OriginUs=NaN;
                        self.Identification=zeros(0,5);
                    case 'id_archive'
                        if numel(tokens)~=7 || ~isfield(self.Record,'generation')
                            error('adaptive:LiveRecord','Identification begin marker missing.');
                        end
                        values=str2double(tokens(2:end));
                        counters=values([1 2 3 6]);
                        if any(~isfinite(values)) || any(counters<0 | counters>2^32-1 | mod(counters,1)~=0) ...
                                || values(1)~=self.Record.generation || ~isfield(self.Record,'intervals') ...
                                || values(2)>self.Record.intervals
                            error('adaptive:LiveRecord','Invalid identification preview row.');
                        end
                        if values(2)==0,self.OriginUs=values(3);end
                        if ~isfinite(self.OriginUs),return;end
                        elapsed=mod(values(3)-self.OriginUs,2^32)/1e6;
                        acknowledgement=NaN;
                        if values(2)>0
                            acknowledgement=mod(values(6)-self.OriginUs,2^32)/1e6;
                            if acknowledgement>elapsed,error('adaptive:LiveTiming','Command publication follows sensor read.');end
                        end
                        if ~isempty(self.Identification) && values(2)<=self.Identification(end,1),return;end
                        self.Identification(end+1,:)=[values(2) elapsed values(4) values(5) acknowledgement];
                        if size(self.Identification,1)>601,self.Identification=self.Identification(end-600:end,:);end
                    case {'id_archive_end','id_archive_abort'},self.addEvent(line);
                    case 'event'
                        self.addEvent(line);
                        if numel(tokens)>1 && strcmp(tokens{2},'boot_auto_identification_downward_stopped_required')
                            self.BootCount=self.BootCount+1;self.Control=zeros(0,20);self.Header={};
                            self.Identification=zeros(0,5);self.LastSequence=NaN;self.LastCycle=NaN;
                            self.Record=struct();self.OriginUs=NaN;self.Configuration=struct();
                            self.CurrentFrame=struct();self.Build=struct();
                            self.Telemetry=struct();self.LatestFit=struct();self.Validation=struct();
                            self.StopEventObserved=false;
                        elseif numel(tokens)>1 && strcmp(tokens{2},'emergency_stop')
                            self.StopEventObserved=true;
                        elseif numel(tokens)>1 && strcmp(tokens{2},'identified_model_nmpc_cycle_start')
                            self.clearStopObservation();
                        end
                end
            catch
                self.MalformedLines=self.MalformedLines+1;
            end
        end
    end
    methods (Static,Access=private)
        function fields=keyValues(tokens)
            fields=struct();
            for index=1:numel(tokens)
                separator=find(tokens{index}=='=',1);if isempty(separator),continue;end
                name=tokens{index}(1:separator-1);if ~isvarname(name),continue;end
                raw=tokens{index}(separator+1:end);number=str2double(raw);
                if isnan(number) && ~strcmpi(raw,'nan'),fields.(name)=raw;
                else,fields.(name)=number;end
            end
        end
    end
end
