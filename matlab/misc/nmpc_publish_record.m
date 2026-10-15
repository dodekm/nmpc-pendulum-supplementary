function nmpc_publish_record(file,writeStaged,verifyStaged,beforePublish,mode)
% Shared verified record transaction. Only explicitly mutable checkpoints use
% mode='replace'; ordinary scientific records use mode='create'. Failed stages
% remain .pending. Atomic replacement must be supported; there is no copy/delete
% fallback that could destroy the previous complete checkpoint on interruption.
validateattributes(file,{'char','string'},{'scalartext','nonempty'});
assert(isa(writeStaged,'function_handle') && isa(verifyStaged,'function_handle'), ...
    'NMPC:RecordCallbacks','Provide staged writer and verifier functions.');
assert(isempty(beforePublish) || isa(beforePublish,'function_handle'), ...
    'NMPC:RecordCallbacks','Invalid pre-publication callback.');
assert(ischar(mode) && ismember(mode,{'create','replace'}), ...
    'NMPC:RecordMode','Use explicit create or replace publication mode.');
file=nmpc_canonical_path(file);staged='';
try
    if isfolder(file) || (strcmp(mode,'create') && isfile(file))
        error('NMPC:RecordCollision','A record already exists: %s.',file);
    end
    folder=fileparts(file);
    if ~isfolder(folder),[okay,message]=mkdir(folder);assert(okay,'NMPC:RecordDirectory','%s',message);end
    parent=javaMethod('get','java.nio.file.Paths',folder,javaArray('java.lang.String',0));
    stage=javaMethod('createTempFile','java.nio.file.Files',parent,'.nmpc-stage-','.pending', ...
        javaArray('java.nio.file.attribute.FileAttribute',0));
    staged=char(stage.toString());
    writeStaged(staged);verifyStaged(staged);
    stream=java.io.RandomAccessFile(staged,'rw');
    closed=onCleanup(@()stream.close());stream.getFD().sync();clear closed
    if ~isempty(beforePublish),beforePublish(staged,file);end
    destination=javaMethod('get','java.nio.file.Paths',file,javaArray('java.lang.String',0));
    options=javaArray('java.nio.file.CopyOption',0);
    if strcmp(mode,'replace')
        % ATOMIC_MOVE intentionally permits replacement on this Windows provider;
        % other providers may reject it, in which case the stage remains recoverable.
        options=javaArray('java.nio.file.CopyOption',1);
        options(1)=java.nio.file.StandardCopyOption.ATOMIC_MOVE;
    end
    javaMethod('move','java.nio.file.Files',stage,destination,options);
catch exception
    failure=MException('NMPC:RecordPersistenceFailure', ...
        'Record publication failed. Mode: %s\nTarget: %s\nStaged file: %s\nCause: %s', ...
        mode,file,staged,exception.message);
    throwAsCaller(addCause(failure,exception));
end
end
