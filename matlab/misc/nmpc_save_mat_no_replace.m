function nmpc_save_mat_no_replace(file,payload,hooks,matVersion)
% Stage, verify and publish one MAT record without replacing an existing file.
% Same-directory publication uses the filesystem no-replace operation, not a
% check-then-save. Failed staging is retained as .pending, not a valid MAT record.
% Hooks are fault-test seams only; callers normally supply file and payload.
if nargin<3,hooks=struct();end
if nargin<4,matVersion='-mat';end
nmpc_save_mat_record(file,payload,'create',matVersion,hooks);
end
