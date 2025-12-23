classdef HelperCollisionChecker < matlab.System
    %HelperCollisionChecker System Object performs the collision checking
    %for all the valid trajectories with the predicted trajectories of
    %MIOs.
    %
    % HelperCollisionChecker system object uses dynamicCapsuleList for
    % collision checking. It updates dynamicCapsuleList with predicted
    % trajectories of MIOs and sampled ego trajectories and returns optimal
    % trajectory. The first valid collision free trajectory is chosen as
    % optimal trajectory.
    %
    % Step Function syntax: function [TrajectoryInfo, TriggerReplan] =
    % stepImpl(obj, ValidTrajectories, EgoAndTargetStates,
    % FutureTrajectory, VehicleProfiles, ReplanFlag) returns the updated
    % trajectory information with collision, evaluation information and
    % optimal trajectory info, TrajectoryInfo, and a flag that indicates
    % requirement for replan, TriggerReplan by taking the information from
    % valid trajectories, ValidTrajectories, ego and target Frenet states,
    % EgoAndTargetStates, predicted trajectories of MIOs, FutureTrajectory,
    % vehicle profiles of target actors, VehicleProfiles, and replan flag,
    % ReplanFlag respectively.
    %
    % For more information see dynamicCapsuleList
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.
    
    % Copyright 2020-2022 The MathWorks, Inc.
    
    %----------------------------------------------------------------------
    % Private Properties
    %----------------------------------------------------------------------
    properties(Access = private)
        %CapList holds the instance of dynamicCapsuleList
        % The CapList stores the instance of dynamic capsule list and is
        % used for collision checking of ego with MIOs in the scenario.
        CapList;
        
        %EgoGeom holds the ego geometry.
        % The EgoGeom stores the geometry of the ego vehicle (Length,Radius
        % and Fixed Transform)
        EgoGeom;
        
        %ActorGeom stores the geometry of target vehicles.
        % The actorGeom stores the geometry of the target vehicle (Length,
        % Radius and Fixed Transformation)
        ActorGeom;
        
        %ActorPoses stores the predicted trajectories of MIOs.
        ActorPoses;
        
        %EgoPoses stores the sampled trajectory of ego vehicle.
        EgoPoses;       
        
        %ExistingTrajectory stores the optimal trajectory.
        % The ExistingTrajectory holds the selected trajectory in the
        % previous replan cycle. This is used to evaluate collision of ego
        % in between replan cycles.
        ExistingTrajectory;
        
        %CollisionFreeTrajIndex holds the optimal trajectory index.
        CollisionFreeTrajIndex;
        
        %NumTrajPoints stores the number of trajectory points in the
        %selected trajectory.
        NumTrajPoints;
        
        %TargetIDs stores the target actor ids.
        TargetIDs;

        % Initialize vehicle dimensions with default values
        CarLen   = 4.7; % in meters
        CarWidth = 1.8; % in meters
        RearAxleRatio = .25;

        %Actor ID of the Ego Vehicle
        %Default value = 1
        EgoActorID = 1;

        %EgoFrontExt defines front extension for the ego vehicle in meters
        % Default Value = 5m;
        EgoFrontExt = 5;
        
        %TargetFrontExt defines front extension for the target vehicle in
        %meters
        % Default Value = 5m;
        TargetFrontExt = 5;
        
        %MaxTimeHorizon holds the maximum value of time horizon in seconds.
        % Maximum time horizon for which the planning has to be done.
        % Default value = 3s
        MaxTimeHorizon = 3;
        
        %TimeResolution defines the time interval between trajectory
        %points in seconds.
        % Default = 0.1s
        TimeResolution = 0.1;
    end
    
    %----------------------------------------------------------------------
    % Main algorithm
    %----------------------------------------------------------------------
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj)
            %setupImpl function performs the one time calculations and
            %Initializations.
            % 
            % The setupImpl function is used to initialize
            % dynamicCapsuleList object, ego and actor dimensions, ego and
            % actor geometry, the actorID list, RefPathFrenet and other
            % variables.
            
            % Create the instance of dynamicCapsuleList and assign it to
            % capList.
            obj.CapList = dynamicCapsuleList;
            
           
            % Calculate maximum number of trajectory points
            maxTrajectoryPoints = obj.MaxTimeHorizon/obj.TimeResolution + 1;
            
            % Set maximum number of trajectory points used for collision checking.
            obj.CapList.MaxNumSteps = maxTrajectoryPoints;

            % Initialize ExistingTrajectory with all zeros.
            obj.ExistingTrajectory=zeros(maxTrajectoryPoints,7);
            
            % Initialize index of collision free trajectory
            obj.CollisionFreeTrajIndex=NaN;
            
            % initialize number of trajectory points with zero.
            obj.NumTrajPoints = 0;
            
            % Get the ego geometry structure from capList.
            [obj.EgoActorID, obj.EgoGeom] = egoGeometry(obj.CapList, obj.EgoActorID);

            % Initialize vehicle dimensions with default values
            obj.CarLen   = 4.7; % in meters
            obj.CarWidth = 1.8; % in meters
            obj.RearAxleRatio = .25;
            
            % Update the ego geometry structure
            obj.EgoGeom.Geometry.Length =  obj.CarLen + obj.EgoFrontExt;
            obj.EgoGeom.Geometry.Radius = obj.CarWidth/2;
            obj.EgoGeom.Geometry.FixedTransform(1,end) = -obj.CarLen*obj.RearAxleRatio;
            
            % Update the capList with the updated ego geometry.
            updateEgoGeometry(obj.CapList, obj.EgoActorID, obj.EgoGeom);
            
            % set maximum number of MIOs to 12.
            maxNumMIOs = 12;
            
            % Initialize actorID with maximum number of MIOs values.
            actorIDs =(1:maxNumMIOs)';
            
            % Initialize target actors geometry
            targetGeom = obj.EgoGeom;
            targetGeom.Geometry.Length = obj.CarLen + obj.TargetFrontExt;           
            obj.ActorGeom = repelem(targetGeom,12,1);
            
            % Update the capList with the updated target geometry.
            updateObstacleGeometry(obj.CapList, actorIDs, obj.ActorGeom);
            
            % Initialize the obstacle and ego poses structure.
            actorposes = struct('States',zeros(maxTrajectoryPoints,3));
            obj.ActorPoses = repelem(actorposes,size(actorIDs, 1),1);
            obj.EgoPoses = actorposes;
        end
        
        %------------------------------------------------------------------
        
        function [TrajectoryInfo,...
                  TriggerReplan] = stepImpl(obj,...
                                            ValidTrajectories,...
                                            EgoAndTargetStates,...
                                            FutureTrajectory,...
                                            VehicleProfiles,...
                                            ReplanFlag,...
                                            PlannerParams, ...
                                            EgoID)
            %stepImpl method executes the main algorithm for the
            % collision checking and generates the optimal trajectory.
            %
            % stepImpl method identifies valid obstacle ids and updates
            % their geometries. It updates predicted trajectories of
            % these valid obstacles(MIOs) for collision checking with valid
            % trajectories of ego vehicle. This outputs optimal trajectory
            % and other information required for downstream blocks.
            
            obj.EgoActorID = EgoID;
            obj.EgoFrontExt = PlannerParams.EgoFrontExt;
            obj.TargetFrontExt = PlannerParams.TargetFrontExt;
            obj.MaxTimeHorizon = max(PlannerParams.TimeHorizon);
            obj.TimeResolution = PlannerParams.TimeResolution;
            
            % Initialize TrajectoryInfo with input trajectory info.
            TrajectoryInfo = ValidTrajectories;
            TriggerReplan  = false;
            collisionFreeTrajectoryFound = false;
            
            if(ReplanFlag)                
                %% Update target list and predictions for collision checking
                % Update valid actor ids and remove invalid ids
                obj.TargetIDs = double(FutureTrajectory.TargetIDs(uint8(1):FutureTrajectory.NumTrajs));                
                invalidActorList = ismember(obj.CapList.ObstacleIDs,obj.TargetIDs);
                validActorIDs = obj.CapList.ObstacleIDs(~invalidActorList);
                if ~isempty(validActorIDs)
                    removeObstacle(obj.CapList, validActorIDs);
                end
                %% Update actor profiles 
                % Use input VehicleProfiles for valid target IDs. Use
                % default vehicle dimensions for invalid target IDs that
                % may come from false tracks.
                obj.ActorGeom(1:FutureTrajectory.NumTrajs,1) = repelem(obj.ActorGeom(1),FutureTrajectory.NumTrajs,1);

                for i=1:numel(VehicleProfiles)
                    if VehicleProfiles(i).ActorID == obj.EgoActorID
                        obj.EgoGeom.Geometry.Length = VehicleProfiles(i).Length + obj.EgoFrontExt;
                        obj.EgoGeom.Geometry.Radius = VehicleProfiles(i).Width/2;
                        obj.EgoGeom.Geometry.FixedTransform(1,end) = -VehicleProfiles(i).Length*obj.RearAxleRatio;
    
                        % Update the capList with the updated ego geometry.
                        updateEgoGeometry(obj.CapList, obj.EgoActorID, obj.EgoGeom);
                    end
                    for j=1:numel(obj.TargetIDs) 
                        if obj.TargetIDs(j) == VehicleProfiles(i).ActorID
                            obj.ActorGeom(j).Geometry.Length = VehicleProfiles(i).Length + obj.TargetFrontExt;
                            obj.ActorGeom(j).Geometry.Radius = VehicleProfiles(i).Width/2;
                            obj.ActorGeom(j).Geometry.FixedTransform(1,end) = -VehicleProfiles(i).Length*0.25;
                        end
                    end
                end

                % Update obstacle geometry in the capsule list
                updateObstacleGeometry(obj.CapList, obj.TargetIDs, obj.ActorGeom(1:numel(obj.TargetIDs),:));
                for i = uint8(1):FutureTrajectory.NumTrajs                    
                    obj.ActorPoses(i).States(uint16(1):FutureTrajectory.Trajectories(i).NumPts,1:3) = FutureTrajectory.Trajectories(i).Trajectory(uint16(1):FutureTrajectory.Trajectories(i).NumPts,1:3);
                end
                
                % Update obstacle poses in the capsule list
                updateObstaclePose(obj.CapList, obj.TargetIDs, obj.ActorPoses(1:FutureTrajectory.NumTrajs));
                
                % Check each valid trajectory for collisions starting with least cost
                for i=1:ValidTrajectories.NumTrajectories
                    numTrajPoints = ValidTrajectories.GlobalTrajectory(i).NumTrajPoints;
                    
                    % Update capsule list with the ego object's candidate trajectory.
                    obj.EgoPoses(1).States(1:numTrajPoints,1:3) = ValidTrajectories.GlobalTrajectory(i).Trajectory(1:numTrajPoints,1:3);
                    updateEgoPose(obj.CapList, obj.EgoActorID, obj.EgoPoses);

                    % Check for collision
                    isColliding = checkCollision(obj.CapList);
                    
                    if all(~isColliding)
                        % If no collisions are found, this is the optimal
                        % trajectory.
                        obj.ExistingTrajectory(1:numTrajPoints,1:6) = ValidTrajectories.GlobalTrajectory(i).Trajectory(1:numTrajPoints,:);
                        obj.ExistingTrajectory(1:numTrajPoints,7) = ValidTrajectories.GlobalTrajectory(i).Times(1:numTrajPoints,:);
                        
                        % Setting the Trajectory Index
                        obj.CollisionFreeTrajIndex = i;
                        
                        % Number of Trajectory points present
                        obj.NumTrajPoints=numTrajPoints;

                        % Indicating that this is a new trajectory     
                        TrajectoryInfo.IsNew = true;
                        
                        % Set IsColliding to false and IsEvaluated to true
                        % as the trajectory is evaluated as a non colliding
                        % trajectory.
                        TrajectoryInfo.GlobalTrajectory(i).IsColliding = false;
                        TrajectoryInfo.GlobalTrajectory(i).IsEvaluated = true;
                        collisionFreeTrajectoryFound = true;
                        break;
                    else
                        % Set IsColliding to true and IsEvaluated to true
                        % as the trajectory is evaluated as a colliding
                        % trajectory.
                        TrajectoryInfo.GlobalTrajectory(i).IsColliding = true;
                        TrajectoryInfo.GlobalTrajectory(i).IsEvaluated = true;
                    end
                end
            else
                %% Check for collision in between replan cycles
                % Get current ego state
                currentEgoState = repelem(EgoAndTargetStates.EgoGlobalState,obj.NumTrajPoints,1);
                
                % Get the index of nearest point on the trajectory
                deltaGlobalPose = obj.ExistingTrajectory(uint16(1):obj.NumTrajPoints,1:6) - currentEgoState;
                distGlobalPose = vecnorm(deltaGlobalPose,2,2);
                [~,idxMin] = min(distGlobalPose);
                
                % Get the forward trajectory points 
                updateTrajectory = obj.ExistingTrajectory(uint16(idxMin):obj.NumTrajPoints,1:6);
                numStates = size(updateTrajectory,1);
                
                % Update ego state in the capsule list 
                ego = struct('States',zeros(numStates,3));
                ego.States(1:numStates,1:3) =  updateTrajectory(:,1:3);
                updateEgoPose(obj.CapList, obj.EgoActorID, ego);
                
                % Check for collision
                isColliding = checkCollision(obj.CapList);
                
                % If there is any collision detected, trigger replan.
                if all(~isColliding)
                    collisionFreeTrajectoryFound = true;
                end
            end
            
            % Trigger replan if there is no collision free trajectory.
            if ~collisionFreeTrajectoryFound
                TrajectoryInfo.GlobalTrajectory(obj.CollisionFreeTrajIndex).Trajectory(1:obj.NumTrajPoints,1:6) = obj.ExistingTrajectory(1:obj.NumTrajPoints,1:6);
                TriggerReplan = true;
            end
            TrajectoryInfo.OptimalTrajectoryIndex = obj.CollisionFreeTrajIndex;
        end
    end
        
    %----------------------------------------------------------------------
    % Common methods
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function s = saveObjectImpl(obj)
            % save object
            s = saveObjectImpl@matlab.System(obj);
        end
        
        %------------------------------------------------------------------
        function loadObjectImpl(obj,s,wasLocked)
            % load object
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink-only methods
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function [TrajectoryInfo,...
                  TriggerReplan]= getOutputSizeImpl(~)
            % Return size for each output port            
            TrajectoryInfo = 1;
            TriggerReplan = 1;
        end
        
        
        %------------------------------------------------------------------
        function [TrajectoryInfo,...
                  TriggerReplan] = getOutputDataTypeImpl(~)
            % Return data type for each output port            
            TrajectoryInfo       = "BusGlobalTrajectories";           
            TriggerReplan         = "boolean";
        end
        
        %------------------------------------------------------------------
        function [TrajectoryInfo,...
                  TriggerReplan] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            TrajectoryInfo       = false;            
            TriggerReplan         = false;
        end
        
        %------------------------------------------------------------------
        function [TrajectoryInfo,...
                  TriggerReplan]=  isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size            
            TrajectoryInfo    = true;
            TriggerReplan = true;
        end        
    end    
end

