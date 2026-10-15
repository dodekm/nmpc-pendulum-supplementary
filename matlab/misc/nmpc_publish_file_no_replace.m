function nmpc_publish_file_no_replace(file,writeStaged,verifyStaged,beforePublish)
% Write, verify, flush and publish a record without replacing any existing file.
% The same-directory no-replace move is the publication boundary. A failed
% transaction retains its .pending stage; it is not a valid published record.
% This is not a guarantee against every filesystem or power-loss failure.
if nargin<4,beforePublish=[];end
nmpc_publish_record(file,writeStaged,verifyStaged,beforePublish,'create');
end
