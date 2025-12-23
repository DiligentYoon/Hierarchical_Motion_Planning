function [mioFrenetStates,...
          leadVehicleFrenetState] =  helperFindMIOs(egoFrenetState,...
                                                    mapDB,...
                                                    targetFrenetStates,...
                                                    targetIDs,...
                                                    initStruct)
%helperFindMIOs function identifies Most Important Objects(MIOs).
%
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
mioFrenetStates = initStruct;

% Get current lane of ego vehicle
curEgoLane = helperDetectLaneNumber(mapDB, egoFrenetState(4));

% Get number of target vehicles
numTargets = size(targetFrenetStates,1);

% Initialize lead vehicle Frenet state
leadVehicleFrenetState = inf(1,6);

%Initialize target lanes
targetLanes = zeros(numTargets,1);
numMIOs = 0;

% Get lane number of target vehicles
for i = 1:numTargets
    targetLanes(i) = helperDetectLaneNumber(mapDB, targetFrenetStates(i,4));    
end

% Ego lane + adjacent lanes
validLanes = curEgoLane + [-1 0 1];
% Remove invalid lanes
validLanes(validLanes<1 | validLanes>mapDB.NumLanes) = []; 

% Calculate target distance from ego
targetDist = targetFrenetStates(:,1) - egoFrenetState(1);
isMIO = false(numTargets,1);

% Identify MIOs and lead vehicle
for i = 1:numel(validLanes)
    targetInLane = targetLanes == validLanes(i); % any targets in the valid lane
    
    % Find all targets in front of ego vehicle 
    distFront = targetDist;
    distFront(distFront<=0 | ~targetInLane) = inf;
    
    % Nearest in front is MIO if distance is not inf
    [val,idx] = min(distFront);
    if ~isinf(val)
        isMIO(idx) = true;
        numMIOs = numMIOs + 1;
        if validLanes(i)==curEgoLane
            leadVehicleFrenetState(1,:) = targetFrenetStates(idx,:); % lead vehicle found            
        end
    end
    
    % Find all targets behind ego vehicle
    distRear = targetDist;
    distRear(distRear>=0 | ~targetInLane) = -inf;
    
    % Nearest in front is MIO if distance is not inf
    [val,idx] = max(distRear);
    if ~isinf(val)
        isMIO(idx) = true;
        numMIOs = numMIOs + 1;
    end    
end

if(numMIOs)
    % Assign mioStates
    mioStates = targetFrenetStates(isMIO,:);
    
    % Assign values to output structure
    mioFrenetStates.NumMIOs = numMIOs;
    mioFrenetStates.TargetIds(1,1:numMIOs) = targetIDs(isMIO);
    mioFrenetStates.MIOStates(1:numMIOs,1:6) = mioStates(1:numMIOs,1:6);
end
end