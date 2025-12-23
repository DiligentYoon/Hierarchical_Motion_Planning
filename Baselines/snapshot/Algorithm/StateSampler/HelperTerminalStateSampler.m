classdef HelperTerminalStateSampler
    %HelperTerminalStateSampler implements methods used in the terminal
    %state sampler block of HighwayLaneChangePlanner.slx Simulink model.
    %
    % The HelperTerminalStateSampler class implements methods to update
    % environment information, cruise control terminal states, lead car
    % following terminal states and lane change terminal states required
    % for the HighwayLaneChangePlanner.slx model.
    %
    %
    %   This is a helper function for example purposes and
    %   may be removed or modified in the future.    
	
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    properties (Access = private)
        % UserPreferredLane holds the value of preferred lane input from
        % the planner parameters
        UserPreferredLane;
    end
    methods
        function obj = HelperTerminalStateSampler()
            % Constructor to initialize the private variable
            obj.UserPreferredLane = 0;
        end
                
        function preferredLane = getUserPreferredLane(obj)
            %getUserPreferredLane returns the value of UserPreferredLane.
            preferredLane = obj.UserPreferredLane;
        end
        
        function obj = setUserPreferredLane(obj, preferredLane)
            %setUserPreferredLane sets the value of UserPreferredLane.
            obj.UserPreferredLane = preferredLane;
        end
        
        function [curEgoLane, preferredLane,...
                distanceToLeadVehicle,...
                leadVehicleTTC] = updateEnvironmentInfo(obj,...
                egoFrenetState,...
                leadVehicleFrenetState,...
                mioFrenetStates,...
                laneInfo,...
                plannerParams,...
                preferredLaneMinus,...
                initStruct)
            %updateEnvironmentInfo updates current, future and preferred lane for ego
            %vehicle along with distance to lead vehicle.
            % The updateEnvironmentInfo updates following information about ego vehicle:
            % 1) Current ego lane.
            % 2) Preferred lane for ego.
            % 3) Distance to lead vehicle.
                        
            % Initialize futureLanes. Sample current, left and right lanes.
            timeHorizons = plannerParams.TimeHorizon;
            
            % Get current lane of ego vehicle
            curEgoLane = helperDetectLaneNumber(laneInfo, egoFrenetState(4));
            
            % Compute distance to lead vehicle
            distanceToLeadVehicle = (leadVehicleFrenetState(1) - egoFrenetState(1)) + (leadVehicleFrenetState(2) - egoFrenetState(2))*timeHorizons;
            
            % Compute TTC to lead vehicle
            leadVehicleTTC = helperCalculateTTC(egoFrenetState,leadVehicleFrenetState);
            
            % Update preferred lane
            [preferredLane, ~]= helperFindPreferredLane(egoFrenetState,mioFrenetStates,...
                                                        laneInfo,plannerParams,...
                                                        preferredLaneMinus,...
                                                        initStruct);
            
        end
        
        function cruiseTerminalStates = cruiseControlSampler(obj,...
                futureLanes,...
                timeHorizons,...
                setSpeed,...
                laneInfo,...
                enableCruiseBehavior,...
                initStruct)
            %cruiseControlSampler samples terminal states for cruise behavior.
            % The cruiseControlSampler samples terminal states for cruise behavior
            % with driver setSpeed.
            
            % Initialize output with the output strut object
            cruiseTerminalStates = initStruct;
            
            % Get lateral offsets possible
            lateralOffsets = laneInfo.LaneCenters(futureLanes);
            
            % Assign values to cruiseStates if cruise behavior is enabled.
            if enableCruiseBehavior
                numStates = size(timeHorizons, 2);
                cruiseTerminalStates.NumStates = numStates;
                cruiseTerminalStates.TerminalStates(1:numStates, 4) = lateralOffsets(:);
                cruiseTerminalStates.TerminalStates(1:numStates, 2) = setSpeed;
                cruiseTerminalStates.TerminalStates(1:numStates, 1) = NaN;
                cruiseTerminalStates.TerminalStates(1:numStates, 7) = timeHorizons';
            end
        end
        
        function lcfTerminalStates = leadCarFollowingSampler(obj,...
                currentEgoLane,...
                timeHorizons,...
                leadVehicleFrenetState,...
                laneInfo,...
                enableLeadCarFollowingBehavior,...
                initStruct)
            %leadCarFollowingSampler samples terminal states for lead car
            %following behavior.
            %
            % The leadCarFollowingSampler samples terminal states for lead car
            % following behavior with the velocity of lead car.
            
            % Initialize output with the output strut object
            lcfTerminalStates = initStruct;
            
            % Get lateral offsets possible
            lateralOffsets = laneInfo.LaneCenters(currentEgoLane);
            
            % Check if there is a lead vehicle
            if all(isinf(leadVehicleFrenetState))
                leadCarExists = false;
            else
                leadCarExists = true;
            end
            
            % Update lcfStates if the lead car following mode is enabled and if there
            % is lead vehicle in ego lane.
            if enableLeadCarFollowingBehavior && leadCarExists
                numStates = size(timeHorizons, 2);
                lcfTerminalStates.NumStates = numStates;
                lcfTerminalStates.TerminalStates(1:numStates,1) = nan;
                lcfTerminalStates.TerminalStates(1:numStates,2) = leadVehicleFrenetState(2);
                lcfTerminalStates.TerminalStates(1:numStates,4) = lateralOffsets(:);
                lcfTerminalStates.TerminalStates(1:numStates,7) = timeHorizons;
            end
        end
        
        function laneChangeStates = laneChangeSampler(obj,...
                mapInfo,...
                currenEgoLane,...
                plannerParams,...
                initStruct)
            %laneChangeSampler samples terminal states for lane change
            %behavior.
            % The laneChangeSampler samples terminal states for lane change
            % behavior using driver setSpeed.
            
            % Initialize output with the output strut object
            laneChangeStates = initStruct;
            
            % Get time horizons for planning
            timeHorizons = plannerParams.TimeHorizon;
            
            % Determine if future lanes are available
            adjacentLanes = currenEgoLane+[-1 1];
            validLanes = adjacentLanes > 0 & adjacentLanes <= mapInfo.NumLanes;
            lateralOffset = mapInfo.LaneCenters(adjacentLanes(validLanes));
            
            % Calculate number of valid adjacent lanes.
            numLane = nnz(validLanes);
            
            % Update laneChangeStates if the lane change behavior is enabled
            if plannerParams.EnableLCBehavior
                numStates = numLane*numel(timeHorizons);
                laneChangeStates.NumStates = numStates;
                laneChangeStates.TerminalStates(1:numStates,1) = NaN;
                laneChangeStates.TerminalStates(1:numStates,2) = plannerParams.SetSpeed;
                laneChangeStates.TerminalStates(1:numStates,4) = repelem(lateralOffset(:),numel(timeHorizons),1);
                laneChangeStates.TerminalStates(1:numStates,7) = repmat(timeHorizons(:),numLane,1);
            end
        end
    end
end

