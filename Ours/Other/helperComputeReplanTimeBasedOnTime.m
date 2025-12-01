function replanFlag = helperComputeReplanTimeBasedOnTime(clock,...
                                                           replanTime,...
                                                           numTrajPoints,...
                                                           DurationTime,...
                                                           times,...
                                                           timeResolution)
% helperComputeCurrentStateBasedOnTime computes the next best point from
% the trajectory based on time.
%
% This function calculates the next best point from the trajectory based on
% time. The output of this block will be used to calculate the inputs for
% pack ego block.
%
%   This is a helper function for example purposes and
%   may be removed or modified in the future.
%
%   Copyright 2020 The MathWorks, Inc.

% Get time delta
timeDelta = clock-replanTime+timeResolution;
% Get time vector
timeVecOut = times(1:numTrajPoints);

if timeDelta > timeVecOut(end)
    timeDelta = timeVecOut(end);
end

% replanFlag = (timeDelta >= (DurationTime - 2*timeResolution));
replanFlag = timeDelta >= DurationTime;
end