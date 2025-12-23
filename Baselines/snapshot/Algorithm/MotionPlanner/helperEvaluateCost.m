function sortedTerminalStates = helperEvaluateCost(terminalStates,...
                                                   plannerParams,...
                                                   laneInfo,...
                                                   speedLimit,...
                                                   preferredLane)
%helperEvaluateCost function evaluates the cost of terminal states and
%outputs sorted terminal states.
%
% The helperEvaluateCost function computes the lateral deviation cost, time
% cost and speed cost of each terminal state and computes the cumulative
% cost. Outputs these costs in sorted order. This function gives preference
% to left lane change hence giving lower cost to the terminal states
% performing left lane change.
%
%   This is a helper function for example purposes and
%   may be removed or modified in the future.

%   Copyright 2020-21 The MathWorks, Inc.

% Initialize and assign the sortedTerminalStates 
sortedTerminalStates = terminalStates;

% Find lateral deviation from nearest lane center
numTerminalStates = terminalStates.NumCombinations;

% Compute lateral deviation from preferred lane
latDeviation = laneInfo.LaneCenters(preferredLane)-terminalStates.Combinations(1:numTerminalStates,4);

% Get states performing left lane change
leftLC = find(latDeviation<0);
latDeviation(leftLC) = latDeviation(leftLC)*(-0.9);

% Calculate lateral deviation cost
latCost = min(latDeviation,[],2)*plannerParams.LatDevWeight;

% Calculate trajectory time cost
timeCost = terminalStates.Combinations(1:numTerminalStates,7)*plannerParams.TimeWeight;

% Calculate terminal speed vs desired speed cost
speedCost = abs(terminalStates.Combinations(1:numTerminalStates,2)- speedLimit)*plannerParams.SpeedWeight;

% Return cumulative cost
costTS = latCost+timeCost+speedCost;

% Determine evaluation order
[sortedCost, idx] = sort(costTS);

% Re-order terminal states in sorted order.
sortedTerminalStates.Combinations(uint8(1):terminalStates.NumCombinations,:) = terminalStates.Combinations(idx,:);
sortedTerminalStates.BehaviorType(uint8(1):terminalStates.NumCombinations,:) = terminalStates.BehaviorType(idx,:);
sortedTerminalStates.Cost(uint8(1):terminalStates.NumCombinations,:) = sortedCost;
end