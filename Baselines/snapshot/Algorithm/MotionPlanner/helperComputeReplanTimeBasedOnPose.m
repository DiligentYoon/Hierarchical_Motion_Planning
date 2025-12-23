function replanFlag = helperComputeReplanTimeBasedOnPose(ego_pose, ...
                                                         numTrajPoints,...
                                                         Trajectory)
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

    replan_dist = 10;
    ego_point = [ego_pose(1), ego_pose(2)];
    end_point = Trajectory(numTrajPoints, 1:2);
    
    dist = hypot(ego_point(1)-end_point(1), ego_point(2)-end_point(2));
    
    if dist <= replan_dist
        replanFlag = true;
    else
        replanFlag = false;
    end


end