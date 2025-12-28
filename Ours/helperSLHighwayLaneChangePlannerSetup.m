function helperSLHighwayLaneChangePlannerSetup(nvp)
% Optional inputs
%   scenarioFcnName:
%     - Name of function which returns scenario which is
%       compatible with HighwayLaneChangeTestBench.slx
%     - Valid values are:
%         "scenario_01_DecisionTrigger"
%         "scenario_02_ClutteredEnv"
%         "scenario_03_SafetyTrigger"
%  This is a helper script for example purposes and may be removed or
%  modified in the future.

%  Copyright 2020-2022 The MathWorks, Inc.

%% Inputs
arguments
    nvp.scenarioFcnName {mustBeMember(nvp.scenarioFcnName,...
        ["scenario_01_MergingCar";...
         "scenario_02_DecisionTrigger"; ...
         ])} = "scenario_02_DecisionTrigger";
end

% Load the test bench model
modelName = "HighwayLaneChangePlannerTestBench";
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

assignin('base', 'scenarioFcnName', nvp.scenarioFcnName);
%% Scenario parameters
%  Call scenario function
scenarioFcnHandle = str2func(nvp.scenarioFcnName);
[scenario, EgoActor, roadCenters] = scenarioFcnHandle();

% Default assessments
assessment.TimeGap = 0.8;
assessment.LongitudinalJerkMax = 5;
assessment.LateralJerkMax =  5;
assessment.frontSafetyGap = 20;
assessment.rearSafetyGap = 5;
assessment.egoTTC = 2;
assessment.nextTTC = 5;

% Assign scenario object and assessment to base workspace
assignin('base', 'scenario', scenario);
assignin('base', 'assessment', assessment);

%% Extract scenario information
% Road center information
globalPlanPoints = roadCenters(:,1:2);

egoInitialPose = struct('ActorID', EgoActor.ActorID,...
                        'Position', EgoActor.Position,...
                        'Velocity', EgoActor.Velocity,...
                        'Acceleration', EgoActor.Acceleration,...
                        'Roll', EgoActor.Roll,...
                        'Pitch', EgoActor.Pitch,...
                        'Yaw', EgoActor.Yaw,...
                        'AngularVelocity', EgoActor.AngularVelocity);

assignin('base', 'egoInitialPose', egoInitialPose);
assignin('base', 'egoActorID', EgoActor.ActorID);

% Ego set speed (m/s)
egoSetVelocity = hypot(EgoActor.Velocity(1), EgoActor.Velocity(2));

%% General Model Parameters
% Simulation sample time (s)
assignin('base', 'Ts', 0.1);

roadCenters = scenario.RoadSegments.RoadCenters;
roadCenters(:,3) = [];

EgoActor = scenario.Actors(1);
currentEgoStates = [EgoActor.Position(1), EgoActor.Position(2), ...
deg2rad(EgoActor.Yaw), 0, norm(EgoActor.Velocity),...
0];

%% Update road network information
% Create referencePathFrenet object to find the initial state of the
% vehicle.
refPathFrenet = referencePathFrenet(roadCenters);
try
    currentFrenetStates = global2frenet(refPathFrenet, currentEgoStates(1:6));
catch ME
    if ME.identifier == 'shared_autonomous:cartesianFrenetConversions:singularity'
        error('Initial orientation of the ego vehicle must align with the direction of travel along the road.');
    end
end

% Get car width and consider it as minimum lane width
minLaneWidth = EgoActor.Width;

mapInfo = getMapInfo(scenario, globalPlanPoints, currentFrenetStates(4),...
                     minLaneWidth);


assignin('base', 'mapInfo', mapInfo);

% Get number of target actors in the scenario
numTargetActors = size(scenario.Actors,2)-1;
assignin('base','numTargetActors',numTargetActors);

%% Define planner parameters
% Define behavior parameters and assign them to base workspace
timeHorizon = 4;
timeResolution = 0.1;
planningResolution = 0.5;
timeStep_sim = 0.01;

lane_width_coeff = 0.25;
lane_change_coeff = 1.0;
change_difficulty_coeff = 1.0;
lane_keeping_coeff = 1.0;

degree = 5;
num_piece = timeHorizon / 0.5;

MaxTTC = 5;
MinTTC = 3;
MaxFront = 20;
MaxRear = 20;

Useresample = true;
feasible_coeff = 0.7;

assignin('base', 'timeHorizon' , timeHorizon);
assignin('base','timeResolution',timeResolution);
assignin('base', 'planningResolution', planningResolution);
assignin('base', 'timeStep_sim', timeStep_sim);

assignin('base', 'lane_width_coeff' , lane_width_coeff);
assignin('base','lane_change_coeff', lane_change_coeff);
assignin('base', 'change_difficulty_coeff', change_difficulty_coeff);
assignin('base', 'lane_keeping_coeff', lane_keeping_coeff);

assignin('base', 'degree', degree);
assignin('base', 'num_piece', num_piece);

assignin('base', 'MaxTTC', MaxTTC);
assignin('base', 'MinTTC', MinTTC);
assignin('base', 'MaxFront', MaxFront);
assignin('base', 'MaxRear', MaxRear);

assignin('base', 'Useresample', Useresample);
assignin('base', 'Feasible_coeff', feasible_coeff);

% Check for set speed and initinal velocity
if floor(egoSetVelocity) ~= 0 || floor(hypot(EgoActor(1).Velocity(1), EgoActor(1).Velocity(2))) ~= 0
    assignin('base','setSpeed',egoSetVelocity);
else
    error('Either set velocity or initial velocity for the ego vehicle must be a real positive scalar to plan a trajectory.'); 
end

%% Feasibility Parameters
assignin('base','maxLonAccel', 3.0);
assignin('base','maxLatAccel', 2.0);
assignin('base', 'maxLonVelocity', 110/3.6);
assignin('base','minLonVelocity',50/3.6);
assignin('base', 'maxLatVelocity', 3.0)
assignin('base', 'minLatVelocity', -3.0);

%% Set compute method for the computation of current state of ego vehicle.
% Use "TimeBased' computation method for the
% HighwayLaneChangePlannerTestBench and "PoseBased" for
% HighwayLaneChangeTestBench model.
assignin('base','computeMethod',ComputeMethod.TimeBased);

% Get actor profiles of target actors.
vehicleProfiles = actorProfiles(scenario);
assignin('base','actorProfiles',vehicleProfiles);

% Define maximum size of bus elements.
maxStatesPerBehavior = 10;
maxBehaviors = 3;
maxTrajectories = maxBehaviors*maxStatesPerBehavior;
maxTrajectoryPoints = max(timeHorizon)/timeResolution + 1;
maxMIOs = 10;
maxGlobalPlanPoints = 10000;
maxNumLanes = 10;
maxTargetActors = 30;
maxGap = 15;
maxControlPoints = num_piece * (degree+1);

assignin('base','maxStatesPerBehavior', maxStatesPerBehavior);
assignin('base','maxTrajectories', maxTrajectories);
assignin('base','maxTrajectoryPoints', maxTrajectoryPoints);
assignin('base','maxMIOs', maxMIOs);
assignin('base','maxGlobalPlanPoints', maxGlobalPlanPoints);
assignin('base','maxNumLanes', maxNumLanes);
assignin('base','maxGap', maxGap);
assignin('base','maxControlPoints', maxControlPoints);
%% Buses Creation  
helperCreateLCPlannerBusObjects(timeHorizon, maxStatesPerBehavior,...
                                maxTrajectories,...
                                maxTrajectoryPoints,...
                                maxNumLanes,...
                                maxGlobalPlanPoints,...
                                maxControlPoints, ...
                                maxMIOs,...
                                maxGap);

evalin('base', sprintf('helperCreateLCBusActorsEgo(%d)', ...
                          length(scenario.actorProfiles) - 1));

evalin('base', sprintf('helperCreateBusPredictedTrajectory(%d,%d)',...
                        maxTargetActors,maxTrajectoryPoints));
end

function laneCenters = calculateLaneCenters(laneWidth,roadCenter)
%calculateLaneCenters computes the lane center distance from road center.

% Get number of lanes
numLanes = length(laneWidth);

% Initialize lane centers
laneCenters = zeros(1,numLanes);
laneCenters(1) = roadCenter - laneWidth(1)/2;

for i = 2:numLanes
    laneCenters(i) = laneCenters(i-1) - (laneWidth(i-1)+laneWidth(i))/2;
end
end

function mapInfo = getMapInfo(scenario, globalPlanPoints, egoLatDist, minLaneWidth)
% Get number of lanes from road network
numLanes = scenario.RoadSegments(1).NumLanes;

laneSpec = scenario.RoadSegments(1).LaneSpecification;
laneWidth = laneSpec.Width;
roadCenter = sum(laneWidth)/2;
laneCenters = calculateLaneCenters(laneWidth, roadCenter);

% Define maximum possible values for lanes and global plan points
maxNumLanes = 10;
maxGlobalPlanPoints = 10000;

% Initialize map data
mapInfo = struct('NumLanes', numLanes,'LaneWidth', zeros(maxNumLanes,1),...
                 'LaneCenters', zeros(maxNumLanes,1),'NumGlobalPlanPoints',...
                 size(globalPlanPoints,1), 'GlobalPlanPoints',...
                 zeros(maxGlobalPlanPoints,2), 'LaneLateralBoundary', zeros(maxNumLanes, 2));

numMarkings = length(laneSpec.Marking);
laneType = laneSpec.Type; % lane type

% Check lane type and update lane center information
for i = 1:numMarkings
    if isequal(laneSpec.Marking(i).Type,LaneBoundaryType.DoubleSolid) % road median?
        if egoLatDist > 0 % ego is on the left road
            laneWidth = laneWidth(1:i-1);
            laneCenters = laneCenters(1:i-1);
            laneType = laneType(1:i-1);
        else
            laneWidth = laneWidth(i:end);
            laneCenters = laneCenters(i:end);
            laneType = laneType(i:end);            
        end
        break;
    end      
end

% Extract driving lane
drivingLane = [laneType.Type] == LaneTypes.Driving & laneWidth>=minLaneWidth;

% Update number of lanes based on valid driving lanes in the scene
numLanes = nnz(drivingLane);
mapInfo.LaneWidth(1:numLanes,:) = laneWidth(drivingLane);
mapInfo.LaneCenters(1:numLanes,:)  = laneCenters(drivingLane);
mapInfo.NumLanes = numLanes;
mapInfo.GlobalPlanPoints(1:mapInfo.NumGlobalPlanPoints,:) = globalPlanPoints;
lane_left  = mapInfo.LaneCenters(1:numLanes) + mapInfo.LaneWidth(1:numLanes)./2;
lane_right = mapInfo.LaneCenters(1:numLanes) - mapInfo.LaneWidth(1:numLanes)./2;
mapInfo.LaneLateralBoundary(1:numLanes, :) = [lane_left, lane_right];
end

