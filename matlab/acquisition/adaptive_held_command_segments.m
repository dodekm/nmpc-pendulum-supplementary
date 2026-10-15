function [time,command]=adaptive_held_command_segments(rows)
%ADAPTIVE_HELD_COMMAND_SEGMENTS Plot only known publication-to-read extents.
% Rows: index, read_elapsed_s, angle_rad, held_speed_command_rad_s, ack_elapsed_s.
% Row0 is a sentinel. Missing predecessor rows must not invent input intervals.
% Separation may leave a small blank command-to-next-command extent: this is
% conservative evidence display, not an assertion that the command became zero.
arguments
    rows (:,5) double
end
keep=rows(:,1)>0 & isfinite(rows(:,2)) & isfinite(rows(:,4)) & isfinite(rows(:,5)) ...
    & rows(:,5)>=0 & rows(:,5)<=rows(:,2);
known=rows(keep,:);
time=reshape([known(:,5) known(:,2) nan(size(known,1),1)]',[],1);
command=reshape([known(:,4) known(:,4) nan(size(known,1),1)]',[],1);
end
