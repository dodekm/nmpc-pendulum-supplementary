function report = reproduce_reference_metrics(repositoryRoot)
%REPRODUCE_REFERENCE_METRICS Offline checks of the two published rig records.
% No serial port, hardware, fitting of shared coefficients, data smoothing,
% figure generation, or file writes. MATLAB R2025b was used for verification.
arguments
    repositoryRoot (1,1) string = string(fileparts(fileparts(fileparts(mfilename('fullpath')))))
end
previousPath = path;
pathCleanup = onCleanup(@()path(previousPath)); %#ok<NASGU>
addpath(fullfile(repositoryRoot,'matlab','misc'));
dataRoot = fullfile(repositoryRoot,'data','manuscript_two_rigs');
checksums = splitlines(string(fileread(fullfile(dataRoot,'SHA256SUMS'))));
checksums = checksums(strlength(checksums)>0);
assert(numel(checksums)==14,'publication:ChecksumManifest','Unexpected data manifest extent.');
for line = checksums'
    entry = regexp(char(line),'^([0-9a-f]{64})  ([A-Za-z0-9_./]+)$','tokens','once');
    assert(~isempty(entry) && ~contains(entry{2},'..'), ...
        'publication:ChecksumManifest','Malformed data checksum entry.');
    assert(strcmpi(nmpc_file_sha256(fullfile(dataRoot,entry{2})),entry{1}), ...
        'publication:DataHash','A published input differs from SHA256SUMS.');
end
names = ["A4988_20261010","HY_DIV268N_20261011"];
hashes = ["7da5515c58043f72adc5583cd1d1e45485dd384fac71923247883910b0d7219e", ...
          "193352aea6af7bbce30df9576ea90a7aadeab678e6432f731649ffc7ecc422e9"];
expectedRows = [1606,1467];
perRig = cell(1,2);
for rig = 1:2
    folder = fullfile(repositoryRoot,'data','manuscript_two_rigs',names(rig));
    assert(strcmpi(nmpc_file_sha256(fullfile(folder,'raw.txt')),hashes(rig)), ...
        'publication:RawHash','The original recording hash does not match.');
    control = readtable(fullfile(folder,'control_cycle_1.csv'));
    assert(height(control)==expectedRows(rig) && all(control.cycle==1) && ...
        all(diff(control.seq)==1) && all(diff(control.tick)==1), ...
        'publication:ControlSelection','Cycle extent or continuity differs.');
    assert(all(isfinite(control{:,{'phi_raw','ref_phi','v_sent','u_sent','solve_us'}}),'all') && ...
        all(control.issued==1),'publication:ControlFinite','Invalid control row.');
    holding = control(control.phase==2,:);
    assert(height(holding)==300,'publication:Holding','Holding must contain 300 samples.');
    vertical = rad2deg(wrap(holding.phi_raw-pi));
    late = holding.t_s>=holding.t_s(end)-9.9-1e-5;
    assert(nnz(late)==100,'publication:LateHold','Late hold must contain 100 samples.');
    model = readtable(fullfile(folder,'identified_model.csv'));
    logged = readtable(fullfile(folder,'validation_metrics.csv'));
    validation = readtable(fullfile(folder,'identification_validation.csv'));
    assert(height(model)==1 && height(logged)==1 && height(validation)==601 && ...
        isequal(validation.index,(0:600)'), ...
        'publication:ValidationSelection','Validation extent or model changed.');
    p = [model.alpha model.beta model.gamma model.bias model.coulomb];
    command = validation.held_speed_rad_s(2:end);
    measured = validation.angle_rad;
    velocity = 0;
    for iteration=1:6
        [prediction,sensitivity] = replay(measured(1),velocity,command(1:5),p,true);
        residual = prediction(2:end)-measured(2:6);
        jacobian = sensitivity(2:end);
        denominator = sum(jacobian.^2);
        assert(isfinite(denominator) && denominator>1e-8, ...
            'publication:Prefix','Singular initial-velocity prefix.');
        velocity = min(10,max(-10,velocity-sum(jacobian.*residual)/denominator));
        assert(isfinite(velocity),'publication:Prefix','Nonfinite prefix velocity.');
    end
    prediction = replay(measured(1),velocity,command,p,false);
    scored = (0:600)'>5;
    residual = prediction(scored)-measured(scored);
    validationRms = sqrt(mean(residual.^2));
    validationPeak = max(abs(residual));
    assert(abs(validationRms-logged.rms)<2e-5 && ...
        abs(validationPeak-logged.peak)<2e-4, ...
        'publication:ReplayAgreement','Frozen replay differs from the MCU validation.');
    capture = find(control.captured==1,1,'first');
    assert(~isempty(capture),'publication:Capture','No recorded capture.');
    complete = find(control.phase==5,1,'first');
    completionTime = NaN;
    if ~isempty(complete), completionTime=control.t_s(complete); end
    metrics = struct('rig',names(rig),'raw_sha256',hashes(rig), ...
        'recorded_samples',height(control),'capture_time_s',control.t_s(capture), ...
        'recorded_endpoint_s',control.t_s(end),'confirmed_rest_time_s',completionTime, ...
        'holding_rms_deg',sqrt(mean(vertical.^2)), ...
        'holding_peak_deg',max(abs(vertical)), ...
        'late_holding_rms_deg',sqrt(mean(vertical(late).^2)), ...
        'late_tracking_rms_deg',sqrt(mean(rad2deg(wrap(holding.phi_raw(late)-holding.ref_phi(late))).^2)), ...
        'late_wheel_command_rms_rad_s',sqrt(mean(holding.v_sent(late).^2)), ...
        'peak_wheel_command_rad_s',max(abs(control.v_sent)), ...
        'peak_acceleration_command_rad_s2',max(abs(control.u_sent)), ...
        'mean_computation_ms',mean(control.solve_us)/1000, ...
        'maximum_computation_ms',max(control.solve_us)/1000, ...
        'converged_fraction',mean(control.converged==1), ...
        'control_fault_samples',nnz(control.control_fault~=0), ...
        'training_rms_deg',rad2deg(model.rms), ...
        'logged_validation_rms_deg',rad2deg(logged.rms), ...
        'logged_validation_peak_deg',rad2deg(logged.peak), ...
        'replayed_validation_rms_deg',rad2deg(validationRms), ...
        'replayed_validation_peak_deg',rad2deg(validationPeak), ...
        'replayed_initial_velocity_rad_s',velocity);
    perRig{rig} = metrics;
end
report = [perRig{:}];
disp(struct2table(report));
end

function [prediction,sensitivity] = replay(angle0,velocity0,commands,p,sensitive)
% Differentiate the same RK4 stages used for the two-state free-flow model.
n = numel(commands);
prediction = zeros(n+1,1);
sensitivity = zeros(n+1,1);
x = [velocity0;angle0;double(sensitive);0];
prediction(1) = angle0;
previous = 0;
dt = .02;
for interval=1:n
    x(1) = x(1)-p(3)*(commands(interval)-previous);
    previous = commands(interval);
    for piece=1:5
        a = derivative(x,p);
        b = derivative(x+.5*dt*a,p);
        c = derivative(x+.5*dt*b,p);
        d = derivative(x+dt*c,p);
        x = x+(dt/6)*(a+2*b+2*c+d);
    end
    assert(all(isfinite(x)) && abs(x(1))<=1000, ...
        'publication:Integration','Frozen-model replay diverged.');
    prediction(interval+1) = x(2);
    sensitivity(interval+1) = x(4);
end
end

function value = derivative(x,p)
friction = tanh(x(1)/.04);
stiffness = p(1)*cos(x(2));
damping = p(2)+p(5)/.04*(1-friction^2);
value = [-p(1)*sin(x(2))-p(2)*x(1)+p(4)-p(5)*friction; ...
    x(1);-stiffness*x(4)-damping*x(3);x(3)];
end

function value = wrap(angle)
value = atan2(sin(angle),cos(angle));
end
