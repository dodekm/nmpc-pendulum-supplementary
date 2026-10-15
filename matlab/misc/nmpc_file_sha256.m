function value=nmpc_file_sha256(file)
% Stream a read-only SHA-256; reject read errors/no progress instead of spinning.
validateattributes(file,{'char','string'},{'scalartext','nonempty'});
[id,message]=fopen(file,'rb');assert(id>=0,'NMPC:HashRead','%s',message);
closed=onCleanup(@()fclose(id)); %#ok<NASGU>
digest=java.security.MessageDigest.getInstance('SHA-256');
assert(fseek(id,0,'eof')==0,'NMPC:HashRead','Cannot inspect file extent.');
extent=ftell(id);assert(isfinite(extent) && extent>=0,'NMPC:HashRead','Invalid file extent.');
assert(fseek(id,0,'bof')==0,'NMPC:HashRead','Cannot rewind file.');
remaining=extent;
% Exact bounded reads avoid MATLAB's nonzero EOF indicator on read-past-end.
% Every requested byte must arrive; a truncated or stalled stream is rejected.
while remaining>0
    requested=min(65536,remaining);bytes=fread(id,requested,'*uint8');[message,status]=ferror(id);
    assert(status==0,'NMPC:HashRead','%s',message);
    assert(numel(bytes)==requested,'NMPC:HashRead','Truncated or stalled file read.');
    digest.update(bytes);remaining=remaining-numel(bytes);
end
assert(fseek(id,0,'eof')==0 && ftell(id)==extent,'NMPC:HashRead','File extent changed during hashing.');
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
