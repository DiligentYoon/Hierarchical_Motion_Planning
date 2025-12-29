function laneNum = helperDetectLaneNumber(mapInfo,latDist)
%helperDetectLaneNumber computes the current lane of the actor.
%
% The helperDetectLaneNumber determines the lane number of the actor based
% on the lane information in the mapInfo,  and the current lateral distance
% of the actor from reference path, latDist. It outputs current lane
% number of the actor, laneNum.
%
%   This is a helper function for example purposes and
%   may be removed or modified in the future.
%
%   Copyright 2020 The MathWorks, Inc.

% Default laneNumber is 0
laneNum = 0;

for i = 1:mapInfo.NumLanes
    halfLaneWidth = mapInfo.LaneWidth(i)/2;
    if all(latDist <= mapInfo.LaneCenters(i)+halfLaneWidth) &&...
            all(latDist >= mapInfo.LaneCenters(i)-halfLaneWidth)
        laneNum = i;
        break;
    end
end

end