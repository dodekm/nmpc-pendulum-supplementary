function [plotTime,indices]=adaptive_gap_indices(sequence,tick,sampleTime,maxInterval)
%ADAPTIVE_GAP_INDICES Linear-time NaN separators, never invented observations.
% Counter wrap is legal; missing/reordered counters or nonforward/long times
% break the curve. Returned indices map only real rows into the plot arrays.
arguments
    sequence (:,1) double
    tick (:,1) double
    sampleTime (:,1) double
    maxInterval (1,1) double {mustBePositive,mustBeFinite} = .15
end
count=numel(sampleTime);
assert(numel(sequence)==count && numel(tick)==count,'adaptive:GapShape','Counter/time lengths differ.');
if count<2,plotTime=sampleTime;indices=(1:count)';return;end
broken=mod(diff(sequence),2^32)~=1 | mod(diff(tick),2^32)~=1 ...
    | diff(sampleTime)<=0 | diff(sampleTime)>maxInterval;
positions=(1:count)'+cumsum([0;broken]);
indices=nan(count+sum(broken),1);plotTime=indices;
indices(positions)=(1:count)';plotTime(positions)=sampleTime;
end
