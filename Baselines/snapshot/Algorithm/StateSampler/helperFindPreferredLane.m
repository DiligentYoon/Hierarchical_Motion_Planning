    function [preferredLane, mioInfo]= helperFindPreferredLane(...
                                                 egoFrenetState,...
                                                 mioFrenetState, road,...
                                                 params,...
                                                 preferredLaneMinus,...
                                                 outputStructObj)
%helperFindPreferredLane updates the preferred lane based on the TTC of
%MIOs with ego vehicle.
%
% The helperFindPreferredLane function calculates TTC of MIOs with ego
% vehicle and updates preferred lane based on safety.
%
%   This is a helper function for example purposes and
%   may be removed or modified in the future.
%

%   Copyright 2020-2022 The MathWorks, Inc.

% default: keep the current preferred lane
preferredLane = preferredLaneMinus;

mioStates = mioFrenetState.MIOStates;

% Get current lane of ego vehicle
curEgoLane = helperDetectLaneNumber(road,egoFrenetState(4));

numMIOs = mioFrenetState.NumMIOs;

% Initialize structure to update MIO information
mioInfo = outputStructObj;

% Update number of MIOs
mioInfo.NumMIOs = numMIOs;

% Check and update MIO information
if numMIOs>0 % MIO exists?   
    for i = 1:numMIOs
        % Get MIO lane
        mioInfo.LaneNum(i) = helperDetectLaneNumber(road,mioStates(i,4));        
        
        % Calculate TTC and other attributes of MIOs with ego vehicle
        [mioInfo.RelativeDist(i),~,mioInfo.RelativeVelo(i),mioInfo.TTC(i),mioInfo.IsFront(i)] = helperCalculateTTC(egoFrenetState,mioStates(i,:));
        
        % Check for safety in each lane
        mioInfo.IsSafe(i) = checkSafety(mioInfo.LaneNum(i),mioInfo.RelativeDist(i),mioInfo.TTC(i),mioInfo.IsFront(i),curEgoLane,params);
    end
    
    % check ego lane
    egoLane = mioInfo.LaneNum==curEgoLane;
    
    if ~isempty(mioInfo.IsSafe(egoLane)) && ... % mio in egoLane exist?
            ~all(mioInfo.IsSafe(egoLane)) % egoLane is unsafe?
        
        if curEgoLane>1 % left lane exists?
            % check left lane
            leftLane = mioInfo.LaneNum==(curEgoLane-1);
            
            if isempty(mioInfo.IsSafe(leftLane)) || ... % leftLane is empty?
                    all(mioInfo.IsSafe(leftLane)) % leftLane is safe?
                preferredLane = curEgoLane-1; % change preferred lane to left lane
                return;
            end
        end
        
        if curEgoLane<road.NumLanes % right lane exists?
            % check right lane
            rightLane = mioInfo.LaneNum==(curEgoLane+1);
            
            if isempty(mioInfo.IsSafe(rightLane)) || ... % rightLane is empty?
                    all(mioInfo.IsSafe(rightLane)) % rightLane is safe?
                preferredLane = curEgoLane+1; % change preferred lane to right lane
                return;
            end
        end
    else
        % If egoLane is safe
        preferredLane = curEgoLane;
    end
end

end

function isSafe = checkSafety(Lane,Range,TTC,Front,EgoLane,params)
%checkSafety function checks for safety of current ego lane, and it's
%adjacent lanes.

% default: current lane is unsafe
isSafe = true;

if Front
    SafetyGap = params.FrontSafetyGap;
else
    SafetyGap = params.RearSafetyGap;
end

if Lane==EgoLane % ego lane
    if Range < SafetyGap || ... % mio is too close?
            (TTC < 0 && ... % mio is approaching
            abs(TTC) < params.EgoTTC) % TTC is too small?
        isSafe = false; % current lane is unsafe
    end
else % left or right lane
    if Range < SafetyGap || ... % mio is too close?
            (TTC < 0 && ... % mio is approaching
            abs(TTC) < params.NextTTC) % TTC is too small?
        isSafe = false; % current lane is unsafe
    end   
end

end