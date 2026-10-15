function nmpc_write_text_record(file,text,mode)
% Common exact UTF-8 serialization for new records and mutable checkpoints.
validateattributes(text,{'char','string'},{'scalartext'});
bytes=unicode2native(char(text),'UTF-8');
writer=@(staged)writeBytes(staged,bytes);verifier=@(staged)verifyBytes(staged,bytes);
if strcmp(mode,'create')
    nmpc_publish_file_no_replace(file,writer,verifier);
else
    nmpc_publish_record(file,writer,verifier,[],mode);
end
end
function writeBytes(file,bytes)
[id,message]=fopen(file,'wb');assert(id>=0,'NMPC:TextOpen','%s',message);
try
    assert(fwrite(id,bytes,'uint8')==numel(bytes),'NMPC:TextWrite','Incomplete staged text write.');
catch exception
    fclose(id);rethrow(exception);
end
assert(fclose(id)==0,'NMPC:TextClose','Cannot close staged text.');
end
function verifyBytes(file,bytes)
id=fopen(file,'rb');assert(id>=0,'NMPC:TextRead','Cannot verify staged text.');
closed=onCleanup(@()fclose(id)); %#ok<NASGU>
assert(isequal(fread(id,Inf,'*uint8'),uint8(bytes(:))), ...
    'NMPC:RecordVerification','Staged UTF-8 text did not round-trip correctly.');
end
