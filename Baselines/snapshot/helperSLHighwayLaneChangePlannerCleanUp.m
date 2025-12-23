% helperSLHighwayLaneChangePlannerCleanUp script clears all workspace
% variables created during execution of this example.

% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2020-22 The MathWorks, Inc.

clearBuses({ ...
    'BusActorProfiles',...
    'BusVehiclePose',...
    'BusActorsEgo',...
    'BusVisualizationInfo',...
    'BusRefPointOnPath',...
    'BusEgoAndTargetStates',...
    'BusGlobalTrajectory',...
    'BusGlobalTrajectories',...
    'BusLaneBoundaries1',...
    'BusLaneBoundaries1LaneBoundaries',...
    'BusMIOFrenetStates',...
    'BusMapInfo',...
    'BusPlannerParams',...
    'BusPredictedTrajectories',...
    'BusPredictedTrajectory',...
    'BusTerminalStates',...
    'BusTerminalStatesCombinations',...
    'BusLaneChangeGap',...
    'BusRelativeMotionCoordinate',...
    'BusEgoAndTargetRelativeStates',...
    'BusMIOInfo'});

clear assessment;
clear egoFrontExt;
clear egoInitialPose;
clear egoTTC;
clear enableCCBehavior;
clear enableLCBehavior;
clear enableLCFBehavior;
clear frontSafetyGap;
clear latDevCost;
clear mapInfo;
clear maxAccel;
clear maxCurvature;
clear maxPlanningHorizon;
clear maxYawRate;
clear minVelocity;
clear nextTTC;
clear numTargetActors;
clear preferredLane;
clear rearSafetyGap;
clear replanRate;
clear setSpeed;
clear speedCost;
clear targetFrontExt;
clear timeCost;
clear timeHorizon;
clear timeResolution;
clear scenario;
clear Ts;
clear computeMethod;
clear egoActorID;
clear maxGlobalPlanPoints;
clear maxMIOs;
clear maxNumLanes;
clear maxStatesPerBehavior;
clear maxTrajectories;
clear maxTrajectoryPoints;
clear scenarioFcnName;
clear timeResolution;
clear actorProfiles;
clear maxGap;
clear velocityHorizon;

% If ans was created by the model; clean it too
if exist('ans','var') && ischar(ans) %#ok<NOANS>
    clear ans
end

function clearBuses(buses)
matlabshared.tracking.internal.DynamicBusUtilities.removeDefinition(buses);
end