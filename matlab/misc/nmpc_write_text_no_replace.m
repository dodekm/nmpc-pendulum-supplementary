function nmpc_write_text_no_replace(file,text)
% Publish exact UTF-8 text through the shared verified no-replace transaction.
nmpc_write_text_record(file,text,'create');
end
