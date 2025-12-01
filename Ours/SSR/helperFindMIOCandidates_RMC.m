function [mioRelativeFrenetStates,...
          mioGlobalFrenetStates] = helperFindMIOCandidates_RMC(egoFrenetState,...
                                                               mapDB,...
                                                               targetRelativeFrenetStates,...
                                                               targetGlobalFrenetStates,...
                                                               targetIDs, EgoLane,...
                                                               initStruct)
% helperFindMIOs function identifies Most Important Objects(MIOs).
% The helperFindMIOs function identifies MIOs in the current and adjacent
% lanes of ego vehicle. This function checks if there is any vehicle exists
% in front and rear of ego vehicle in it's current lane and adjacent right
% and left lanes.
%
%   This is a helper function for example purposes and
%   may be removed or modified in the future.
%

%   Copyright 2020-2021 The MathWorks, Inc.

% Initialize output structure
mioRelativeFrenetStates = initStruct;
mioGlobalFrenetStates = initStruct;
% Get current lane of ego vehicle
curEgoLane = EgoLane;

% Get number of target vehicles
numTargets = size(targetRelativeFrenetStates,1);

%Initialize target lanes
targetLanes = zeros(numTargets,1);
numMIOs = 0;

% Get lane number of target vehicles
for i = 1:numTargets
    targetLanes(i) = helperDetectLaneNumber(mapDB, targetGlobalFrenetStates(i,4));    
end

% Ego lane + adjacent lanes (자차 기준 왼 & 오 차선)
validLanes = curEgoLane + [1 0 -1];
% Remove invalid lanes
validLanes(validLanes < 1 | validLanes > mapDB.NumLanes) = []; 

% Calculate target distance from ego (Relative)
targetDist = targetRelativeFrenetStates(:,1);
isMIO = false(numTargets,1);

for i = 1:numel(validLanes)
    % 유효한 차선에서만 검사 -> 건너편 차선 X
    % 유효 차선 차량 중 앞, 뒤로 가장 가까운 차량만 선별 (한 차선 당, MIO는 최대 2개를 넘지 못함)
    targetInLane = targetLanes == validLanes(i);

    distFront = targetDist;
    distFront(distFront <= 0 | distFront > 150 | ~targetInLane) = inf;

    [min_dist, idx] = min(abs(distFront));
    if ~isinf(min_dist)
        isMIO(idx) = true;
        numMIOs = numMIOs + 1;
    end

    distRear = targetDist;
    distRear(distRear >= 0 | distRear < -150 | ~targetInLane) = -inf;
    [min_dist, idx] = min(abs(distRear));
    if ~isinf(min_dist)
        isMIO(idx) = true;
        numMIOs = numMIOs + 1;
    end

if(numMIOs)
    % Assign mioStates
    miorelativeStates = targetRelativeFrenetStates(isMIO,:);
    mioglobalStates = targetGlobalFrenetStates(isMIO,:);
    
    % Assign values to output structure
    mioRelativeFrenetStates.NumMIOs = numMIOs;
    mioRelativeFrenetStates.TargetIds(1,1:numMIOs) = targetIDs(isMIO);
    mioRelativeFrenetStates.MIOStates(1:numMIOs,1:6) = miorelativeStates(1:numMIOs,1:6);

    mioGlobalFrenetStates.NumMIOs = numMIOs;
    mioGlobalFrenetStates.TargetIds(1,1:numMIOs) = targetIDs(isMIO);
    mioGlobalFrenetStates.MIOStates(1:numMIOs,1:6) = mioglobalStates(1:numMIOs, 1:6);
end
end