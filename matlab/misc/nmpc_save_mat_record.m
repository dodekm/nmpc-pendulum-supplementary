function nmpc_save_mat_record(file,payload,mode,matVersion,hooks)
% Common serialization/verification for immutable records and mutable checkpoints.
validateattributes(payload,{'struct'},{'scalar'});
validateattributes(hooks,{'struct'},{'scalar'});
assert(ischar(matVersion) && ismember(matVersion,{'-mat','-v7.3'}), ...
    'NMPC:RecordFormat','Supported MAT formats are -mat and -v7.3.');
assert(isempty(setdiff(fieldnames(hooks),{'write','before_publish'})), ...
    'NMPC:RecordHooks','Unknown persistence fault-test hook.');
for field=fieldnames(hooks)'
    assert(isa(hooks.(field{1}),'function_handle'),'NMPC:RecordHooks','Hooks must be function handles.');
end
writer=@(staged)savePayload(staged,payload,matVersion);
if isfield(hooks,'write'),writer=@(staged)hooks.write(staged,payload);end
before=[];if isfield(hooks,'before_publish'),before=hooks.before_publish;end
nmpc_publish_record(file,writer,@(staged)verifyPayload(staged,payload),before,mode);
end
function savePayload(staged,payload,matVersion),save(staged,'-struct','payload',matVersion);end
function verifyPayload(staged,payload)
recovered=load(staged,'-mat');
assert(isequaln(orderfields(recovered),orderfields(payload)), ...
    'NMPC:RecordVerification','Staged record did not round-trip correctly.');
end
