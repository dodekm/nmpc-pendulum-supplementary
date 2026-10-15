function directory=adaptive_create_session_directory(parent,name)
% Exclusively reserve a new capture directory; existence checks are not locks.
validateattributes(parent,{'char','string'},{'scalartext','nonempty'});
validateattributes(name,{'char','string'},{'scalartext','nonempty'});
assert(~isempty(regexp(char(name),'^adaptive_serial_[A-Za-z0-9_-]+$','once')), ...
    'adaptive:SessionName','Use a plain adaptive_serial_ session name.');
parent=nmpc_canonical_path(parent);
if ~isfolder(parent),[okay,message]=mkdir(parent);assert(okay,'adaptive:CaptureDirectory','%s',message);end
directory=fullfile(parent,char(name));
try
    target=javaMethod('get','java.nio.file.Paths',directory,javaArray('java.lang.String',0));
    javaMethod('createDirectory','java.nio.file.Files',target, ...
        javaArray('java.nio.file.attribute.FileAttribute',0));
catch exception
    failure=MException('adaptive:CaptureDirectory', ...
        'Cannot exclusively reserve a new session: %s. %s',directory,exception.message);
    throwAsCaller(addCause(failure,exception));
end
end
