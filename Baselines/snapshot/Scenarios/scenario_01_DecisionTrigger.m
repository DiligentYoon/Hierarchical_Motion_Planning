function [scenario, egoVehicle, roadCenters] = scenario_LC_06_DecisionTrigger()
%scenario_06_DoubleLaneChange creates a scenario that is compatible
% with HighwayLaneChangeTestBench.slx. This test scenario is on a Straight
% road. There are ten other vehicles in the scenario. Two
% vehicles(leftRear1 and leftRear2) are configured to perform lane change
% and all other vehicles follow their lanes. Ego vehicle initiates lane
% change to avoid these vehicles twice.
%
% This scenario is created in Driving Scenario Designer and exported as a
% MATLAB function.

%   Copyright 2019-2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 50, ...
    'SampleTime', 0.1);

% Add all road segments
roadCenters = [0 3.6 0;
    1400 3.6 0];
laneSpecification = lanespec(4);
straightRoad = road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [200 1.8 0]);
waypoints = [200 1.8 0;
    240.5 1.8 0;
    267 1.8 0;
    268 1.8 0;
    297 5.4 0;
    298 5.4 0;
    359 5.4 0;
    360 5.4 0;
    400 9 0;
    401 9 0;
    1400 9 0];
speed = [15;15;15;15;20;20;20;20;26;26;26];
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
egoFront1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [290 1.8 0]);
waypoints = [290 1.8 0;
    300 1.8 0;
    340 1.8 0;
    400 1.8 0;
    500 1.8 0;
    550 1.8 0;
    577.8 1.7 0;
    802.2 1.7 0;
    1147.8 1.4 0;
    1400.9 1.7 0];
speed = [10;10;10;15;18;19;19;19;19;19];
waittime = [0;0;0;0;0;0;0;0;0;0];
trajectory(egoFront1, waypoints, speed, waittime);

leftRear1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [140 5.4 0]);
waypoints = [140 5.4 0;
    360 5.4 0;
    361 5.4 0;
    410 9 0;
    411 9 0;
    1400 9 0];
speed = 34;
trajectory(leftRear1, waypoints, speed);

leftFront1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [304.7 5.4 0]);
waypoints = [304.7 5.4 0;
    370.2 5.4 0;
    650 5.4 0;
    700 5.4 0;
    751 5.4 0;
    1400 5.4 0];
speed = [17;17;17;17;17;17];
trajectory(leftFront1, waypoints, speed);

right1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [186 -1.8 0]);
waypoints = [186 -1.8 0;
    250.4 -1.8 0;
    1400 -1.8 0];
speed = 12;
trajectory(right1, waypoints, speed);

right2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [222 -1.8 0]);
waypoints = [222 -1.8 0;
    286 -1.8 0;
    1400 -1.8 0];
speed = 12;
trajectory(right2, waypoints, speed);

right3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [258 -1.8 0]);
waypoints = [258 -1.8 0;
    332.3 -1.8 0;
    1400 -1.8 0];
speed = 12;
trajectory(right3, waypoints, speed);

right4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [294 -1.8 0]);
waypoints = [294 -1.8 0;
    340.6 -1.8 0;
    1400 -1.9 0];
speed = 12;
trajectory(right4, waypoints, speed);

leftRear2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [100 5.4 0]);
waypoints = [100 5.4 0;
    640 5.4 0;
    641 5.4 0;
    680 9 0;
    681 9 0;
    1400 9 0];
speed = 20;
trajectory(leftRear2, waypoints, speed);

farLeft1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [10 9 0]);
waypoints = [60 9 0;
    1400 9 0];
speed = 22;
trajectory(farLeft1, waypoints, speed);

farLeft2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [100 9 0]);
waypoints = [100 9 0;
    1400 9 0];
speed = 22;
trajectory(farLeft2, waypoints, speed);

% Output road centers
roadCenters = straightRoad.RoadCenters;
end

