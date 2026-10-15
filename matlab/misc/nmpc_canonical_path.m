function resolved=nmpc_canonical_path(value,baseDirectory)
% Resolve relative names against MATLAB's caller directory, not JVM user.dir.
if nargin<2,baseDirectory=pwd;end
validateattributes(value,{'char','string'},{'scalartext','nonempty'});
validateattributes(baseDirectory,{'char','string'},{'scalartext','nonempty'});
value=char(value);baseDirectory=char(baseDirectory);
if ~java.io.File(value).isAbsolute()
    assert(java.io.File(baseDirectory).isAbsolute(),'NMPC:PathBase','Base directory must be absolute.');
    assert(isempty(regexp(value,'^[A-Za-z]:','once')), ...
        'NMPC:DriveRelativePath','Ambiguous drive-relative paths are unsupported.');
    value=fullfile(baseDirectory,value);
end
resolved=char(java.io.File(value).getCanonicalPath());
end
