 classdef HelperPlotLCPlannerStatus < matlab.System
    %HelperPlotLCPlannerStatus helps in visualizing the execution of
    %trajectory on chase view and top view
    %   using MATLAB Chase plot for the ego vehicle
    %     This System object helps in visualizing
    %     1. Sampled trajectories and their status.
    %     2. Optimal trajectory.
    %     3. Capsules used for collision checking.
    %     4. Other target vehicles in the scenario.
    %
    %   NOTE: This is a helper function for example purposes and
    %   may be removed or modified in the future.

    %   Copyright 2019-2022 The MathWorks, Inc.
    
    % Private properties of the System object
    properties(Access = protected)
        % Figure holds the instance of MATLAB figure and their properties.
        Figure;
        
        % HPanelTop holds the instance of panel in the figure that shows
        % top view of the scene.
        HPanelTop;
        
        % HPanelChase holds the instance of panel in the figure that shows
        % chase view of the scene.
        HPanelChase;
        
        % HAxisTop holds the instance of axis in the figure that shows
        % top view of the scene.
        HAxisTop;
        
        % HAxisChase holds the instance of axis in the figure that shows
        % chase view of the scene.
        HAxisChase;
        
        % LineHandlesTopView holds the line handles for top view.
        LineHandlesTopView;
        
        % LineHandlesChase holds the line handles for chase view.
        LineHandlesChase;
        
        % ScenarioObj holds the instance of drivingScenario.
        ScenarioObj;
        
        % Capsule list holds the instance of dynamicCapsulList.
        CapList;
        
        % Capsule list holds the instance of dynamicCapsulList.
        CapListTopView;
        
        %EgoGeom holds the ego vehicle geometry.
        EgoGeom;
        
        %ActorGeom holds the geometry of a vehicle that is used to define
        %the default vehicle geometry.
        ActorGeom;
        
        %ActorPoses holds the poses of obstacles and is used to update the
        %actor poses in the dynamicCapsuleList.
        ActorPoses;
        
        %EgoPoses holds the poses of ego vehicle and is used to update the
        %ego pose in the dynamicCapsuleList.
        EgoPoses;
        
        %TargetIDs holds the ids of MIOs for updating the
        %dynamicCapsuleList.
        TargetIDs;
        
        %TargetFrontExt holds target front extension.
        TargetFrontExt;
        
        %EgoFrontExt holds ego front extension.
        EgoFrontExt;
        
        %EgoActorID holds the actor id of ego vehicle.
        EgoActorID;

        % GIF
        GifFilename;
        FrameCount;

        % velocity handle
        EgoVelTextTop
        VelTextHandlesTop

        EgoTextTformTop
    end

    properties(Nontunable)
        %EnableChaseView Enable Chase view
        EnableChaseView (1, 1) logical = false;
        
        %EnableCaplistChaseView Enable Capsule List on Chase view
        EnableCaplistChaseView (1, 1) logical = false;
        
        %EnableTrajChase Enable Trajectories on Chase view
        EnableTrajChase (1, 1) logical = false;

        %EnableTopView Enable Top View
        EnableTopView (1, 1) logical = true;
        
        %EnableTrajTopView Enable Trajectories on Top view
        EnableTrajTopView (1, 1) logical = true; 

        %EnableBirdsEyeView
        EnableBirdsEyeView (1, 1) logical = true;

        % gif
        useGIF (1, 1) logical = false;

    end

    % Constants used in the System object.
    properties(Constant, Hidden)

        % Length of the car
        CarLength = 4.7;
        
        % Width of the car
        CarWidth = 1.8;
        
        % OriginOffset defines the distance between rear axle and the rear
        % end of the vehicle
        OriginOffset = [-1.35 0];
        
        % Color code blue
        ColorBlue  = [0 0.447 0.741];
        
        % Max number of points in each trajectory
        MaxTrajPoints = 41;
        
        % Maximum number of trajectories
        MaxTrajectories = 20;

        % Screen Specification
        Width = 1920;
        Height = 1280;

    end
    
    %----------------------------------------------------------------------
    % Main algorithm
    %----------------------------------------------------------------------
    methods
        function obj = HelperPlotLCPlannerStatus(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
            if ~obj.EnableChaseView
                obj.EnableTrajChase = false;
                obj.EnableCaplistChaseView = false;
            end
            if ~obj.EnableTopView
                obj.EnableTrajTopView = false;   
            end
        end
    end
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj)
            %setupImpl Initializes the properties of System object.
            % The setupImpl method initializes the Figure, chase view,
            % top view,line handle and legend for both top view and chase
            % view.
            
            % Create and set Figure Properties
            figureName = 'Lane Change Status Plot';            
            obj.GifFilename = 'test.gif';
            obj.Figure = findobj('Type','Figure','Name',figureName);
            obj.FrameCount = 0;
            if isempty(obj.Figure)
                % screenSize = double(get(groot,'ScreenSize'));
                screenSize = double([1, 1, obj.Width, obj.Height]);
                obj.Figure = figure('Name',figureName);
                obj.Figure.Position = [screenSize(3)*0.17 screenSize(4)*0.15 screenSize(3)*0.6 screenSize(4)*0.6];
                obj.Figure.NumberTitle = 'off';
                obj.Figure.MenuBar = 'none';
                obj.Figure.ToolBar = 'none';
            end
            % Clear figure
            clf(obj.Figure);
            
            % Set the visibility of figure
            if ~obj.EnableChaseView && ~obj.EnableTopView
                % Disabling the visualization window
                set(obj.Figure,"Visible","off");
            end
            
            % Read information from base workspace
            egoInitialPose = evalin('base','egoInitialPose');
            obj.EgoActorID = egoInitialPose.ActorID;
            % obj.EgoFrontExt = evalin('base','egoFrontExt');
            % obj.TargetFrontExt = evalin('base','targetFrontExt');
            obj.ScenarioObj = evalin('base', 'scenario');
            %% Initialize capsule list
            obj.CapList = dynamicCapsuleList;
            
            % Updating the EgoGeometry in Capsule List
            [obj.EgoActorID, obj.EgoGeom] = egoGeometry(obj.CapList, obj.EgoActorID);
            obj.EgoGeom.Geometry.Length = obj.CarLength; % in meters
            obj.EgoGeom.Geometry.Radius = obj.CarWidth/2; % in meters
            obj.EgoGeom.Geometry.FixedTransform(1,end) = 1;
            updateEgoGeometry(obj.CapList, obj.EgoActorID, obj.EgoGeom);
            
            % Initialize Capsule List with default actor ids and default
            % actor geometry
            actorID=(1:12)';
            targetGeom = obj.EgoGeom;
            targetGeom.Geometry.Length = obj.CarLength;
            obj.ActorGeom = repelem(targetGeom,12,1);
            updateObstacleGeometry(obj.CapList, actorID, obj.ActorGeom);
            
            % Initializing the obstacle and ego poses
            [~, obj.ActorPoses] = obstaclePose(obj.CapList, obj.CapList.ObstacleIDs);
            [obj.EgoActorID, obj.EgoPoses] = egoPose(obj.CapList, obj.CapList.EgoIDs);
            
            
            % Initializing the obstacle and ego poses
            [~, obj.ActorPoses] = obstaclePose(obj.CapList, obj.CapList.ObstacleIDs);
            [obj.EgoActorID, obj.EgoPoses] = egoPose(obj.CapList, obj.CapList.EgoIDs);
            
            % Get ego actor information from scenario object
            egoCar = obj.ScenarioObj.Actors(obj.EgoActorID);
            
            % Initialize chase view and top view
            if obj.EnableTopView && obj.EnableChaseView

                % Create axis handle for top view
                obj.HPanelTop = uipanel(obj.Figure,'Units','Normalized','Position',[2/3 0 1/3 1],'Title','Top View');
                obj.HAxisTop = axes('Parent',obj.HPanelTop);
                
                % Create chase view and set properties to turn it into top
                % view
                chasePlot(egoCar,'Parent',obj.HAxisTop, ...
                    'Centerline','off',...
                    'ViewHeight',   70, ...
                    'ViewLocation', [20, 0], ...
                    'ViewPitch',    90); 

                hold(obj.HAxisTop,'on');
                % Add axis tool bar for top view
                axtoolbar(obj.HAxisTop, {'rotate','pan','zoomin','zoomout','restoreview'});
                hold(obj.HAxisTop,'off');
                % Create axis handle for chase view
                obj.HPanelChase = uipanel(obj.Figure,'Units','Normalized','Position',[0 0 2/3 1],'Title','Chase View');
                obj.HAxisChase = axes('Parent',obj.HPanelChase);

                % Create chase view for the ego vehicle
                chasePlot(egoCar, 'Parent', obj.HAxisChase, 'ViewLocation',-[obj.CarLength*4, 0],'ViewHeight',11,'ViewPitch',20);
                
                hold(obj.HAxisChase,'on');
                % Add axes tool bar for chase view
                axtoolbar(obj.HAxisChase, {'rotate','pan','zoomin','zoomout','restoreview'});
                hold(obj.HAxisChase,'off');
                
            elseif obj.EnableChaseView

                obj.HPanelChase = uipanel(obj.Figure,'Units','Normalized','Position',[0 0 1 1],'Title','Chase View');
                
                % Get axes for chase view
                obj.HAxisChase = axes('Parent',obj.HPanelChase);

                % Create chase view for the ego vehicle
                chasePlot(egoCar,'Parent',obj.HAxisChase,...
                    'ViewLocation',-[obj.CarLength*4, 0],...
                    'ViewHeight',11,...
                    'ViewYaw',0,...
                    'ViewPitch',20,...);
                    'ViewRoll',0);
                
                % Add axes tool bar for chase view
                axtoolbar(obj.HAxisChase, {'rotate','pan','zoomin','zoomout','restoreview'});
            else
                obj.HPanelTop = uipanel(obj.Figure,'Units','Normalized','Position',[0 0 1 1],'Title','Top View');
                obj.HAxisTop = axes('Parent',obj.HPanelTop);
                
                % Create chase view and set properties to turn it into top
                % view
                chasePlot(egoCar,'Parent',obj.HAxisTop, ...
                    'Centerline','off',...
                    'ViewHeight',   300, ...
                    'ViewLocation', [20, 0], ...
                    'ViewYaw',0,...
                    'ViewPitch',    90);
                
                % Add axis tool bar for top view
                axtoolbar(obj.HAxisTop, {'rotate','pan','zoomin','zoomout','restoreview'});
            end
            
            %% Create legend for the candidate trajectories in the plot
            % Legend for chase view
            if obj.EnableChaseView && obj.EnableTrajChase
                hChase(1) = line(obj.HAxisChase,0,0,...
                    'LineStyle','-','Color','g','LineWidth',3);
                set(hChase(1),'XData',[],'YData',[]);
                
                hChase(2) = line(obj.HAxisChase,0,0,...
                    'LineStyle','-','Color','k','LineWidth',2);
                set(hChase(2),'XData',[],'YData',[]);
                
                legend(obj.HAxisChase,hChase,...
                    {'optimal','not selected'},...
                    'Color',[0.94,0.94,0.94],...
                    'AutoUpdate','off');
            end
            
            % Legend for top view
            if obj.EnableTopView && obj.EnableTrajTopView
                hTop(1) = line(obj.HAxisTop,0,0,...
                    'LineStyle','-','Color','g','LineWidth',3);
                set(hTop(1),'XData',[],'YData',[]);
                hTop(2) = line(obj.HAxisTop,0,0,...
                    'LineStyle','-','Color','k','LineWidth',2);
                set(hTop(2),'XData',[],'YData',[]);

                legend(obj.HAxisTop,hTop,...
                    {'optimal','not selected'},...
                    'Color',[0.94,0.94,0.94],...
                    'AutoUpdate','off');
            end
            
            % Initialize line handles to plot candidate trajectories in top
            % view and chase view
            obj.VelTextHandlesTop = gobjects(0);
            obj.LineHandlesTopView = [];
            obj.LineHandlesChase = [];
            if isempty(obj.LineHandlesTopView)
                for i = 1:obj.MaxTrajectories
                    if obj.EnableTopView && obj.EnableTrajTopView
                        obj.LineHandlesTopView(end+1) = line('Parent',obj.HAxisTop);
                    end
                    if obj.EnableChaseView && obj.EnableTrajChase
                        obj.LineHandlesChase(end+1) = line('Parent',obj.HAxisChase);
                    end
                end
            end

            obj.EgoTextTformTop = hgtransform('Parent', obj.HAxisTop, 'Tag','EgoSpeedTformTop');
            ahead = (obj.CarLength*0.5) + 2.0;                     
            obj.EgoVelTextTop = text(...
                'Parent', obj.EgoTextTformTop, ...
                'Position', [ahead, 0, 0], ...   
                'String', '0.0 km/h', ...
                'Color', [0 0 1], ...
                'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', ...
                'Clipping', 'on', ...
                'Tag', 'EgoSpeedTextTop');

            % Clone CapList to CapListTopView;
            obj.CapListTopView = obj.CapList;
        end
        
        %------------------------------------------------------------------
        function stepImpl(obj, varargin)

            %stepImpl implements the core logic for visualization at every
            %simulation step
            %
            %   The stepImpl implements the logic to plot capsule list and
            %   sampled trajectories on both top view and chase view of the
            %   MATLAB figure.
            TargetActorsWorld = varargin{1};
            VisualizationInfo = varargin{2};
            ObstacleProfiles = varargin{3};
            EgoActor = varargin{4};

            if obj.EnableChaseView || obj.EnableTopView

                % Extract the input information
                futureTrajectory      = VisualizationInfo.FutureTrajectories;
                GlobalTraj  = VisualizationInfo.TrajectoryInfo;
                OptimalTrajIdx = uint8(GlobalTraj.OptimalTrajectoryIndex);
                numTrajectories = GlobalTraj.NumTrajectories;
                
                %% Plot sampled trajectories on top view and chase view.
                
                % Plot optimal trajectory in green color
                if OptimalTrajIdx ~= 0
                    width = 1;
                    widthChase = 4;
                    color = 'g';
                    lineStyle = '-';
                    traj = GlobalTraj.GlobalTrajectory(OptimalTrajIdx).Trajectory;
                    numTrajPts = GlobalTraj.GlobalTrajectory(OptimalTrajIdx).NumTrajPoints;
                    
                    % Get the ego index on the trajetory.
                    [~, egoIndex] = min(vecnorm(traj(1:numTrajPts,1:2)-EgoActor.Position(1:2),2,2));

                    % Update optimal trajectory on top view
                    if obj.EnableTopView && obj.EnableTrajTopView
                        set(obj.LineHandlesTopView(1),'XData',traj(egoIndex:numTrajPts,1),'YData',traj(egoIndex:numTrajPts,2),'LineStyle',lineStyle,'Color',color,'LineWidth',width);
                    end
                    
                    % Update optimal trajectory on Chase view
                    if obj.EnableChaseView && obj.EnableTrajChase
                        set(obj.LineHandlesChase(1),'XData',traj(egoIndex:numTrajPts,1),'YData',traj(egoIndex:numTrajPts,2),'LineStyle',lineStyle,'Color',color,'LineWidth',widthChase);
                    end
                else
                    % Update optimal trajectory on top view
                    if obj.EnableTopView && obj.EnableTrajTopView
                        set(obj.LineHandlesTopView(1),'XData',[],'YData',[]);
                    end
                    
                    % Update optimal trajectory on Chase view
                    if obj.EnableChaseView && obj.EnableTrajChase
                        set(obj.LineHandlesChase(1),'XData',[],'YData',[]);
                    end
                end
                
                % Update color of sampled trajectory based of trajectory info
                % and plot them on both top view and chase view.
                width = 0.75;
                widthChase = 2;
                validTrajIdx = 0;
                handleIndex = 2;
                for i = 1:obj.MaxTrajectories
                    if i <= numTrajectories
                        traj = GlobalTraj.GlobalTrajectory(i).Trajectory;
                        numTrajPts = GlobalTraj.GlobalTrajectory(i).NumTrajPoints;
                        if GlobalTraj.GlobalTrajectory(i).IsValid
                            validTrajIdx = validTrajIdx + 1;
                            lineStyle = '-';
                            if ~GlobalTraj.GlobalTrajectory(i).IsEvaluated
                                % Trajectory did not get selected
                                color = 'k';
                            end
                        else
                            % Trajectory violated a constraint, and therefore was never
                            % evaluated
                            lineStyle = '--';
                            color = 'c';
                        end
                        if OptimalTrajIdx==0 || i ~= OptimalTrajIdx
                            % Update sampled trajectories on top view
                            if obj.EnableTopView && obj.EnableTrajTopView
                                set(obj.LineHandlesTopView(handleIndex),'XData',traj(1:numTrajPts,1),'YData',traj(1:numTrajPts,2),'LineStyle',lineStyle,'Color',color,'LineWidth',width);
                            end
                            
                            % Update sampled trajectories on chase view
                            % if obj.EnableChaseView && obj.EnableTrajChase
                            %     set(obj.LineHandlesChase(handleIndex),'XData',traj(1:numTrajPts,1),'YData',traj(1:numTrajPts,2),'LineStyle',lineStyle,'Color',color,'LineWidth',widthChase);
                            % end
                            handleIndex = handleIndex + 1;
                        end
                    else
                        % Update sampled trajectories on top view
                        if obj.EnableTopView && obj.EnableTrajTopView
                            set(obj.LineHandlesTopView(i),'XData',[],'YData',[]);
                        end
                        % Update sampled trajectories on chase view
                        % if obj.EnableChaseView && obj.EnableTrajChase
                        %     set(obj.LineHandlesChase(i),'XData',[],'YData',[]);
                        % end
                    end
                end
                
                drawnow
                
                % Update actor info to scenario object for plotting
                obj.ScenarioObj.Actors(1).Position = EgoActor.Position;
                obj.ScenarioObj.Actors(1).Velocity = EgoActor.Velocity;
                obj.ScenarioObj.Actors(1).Roll = EgoActor.Roll;
                obj.ScenarioObj.Actors(1).Pitch = EgoActor.Pitch;
                obj.ScenarioObj.Actors(1).Yaw = EgoActor.Yaw;
                obj.ScenarioObj.Actors(1).AngularVelocity = EgoActor.AngularVelocity;
                
                for i=1:TargetActorsWorld.NumActors
                    obj.ScenarioObj.Actors(i+1).Position = TargetActorsWorld.Actors(i).Position;
                    obj.ScenarioObj.Actors(i+1).Velocity = TargetActorsWorld.Actors(i).Velocity;
                    obj.ScenarioObj.Actors(i+1).Roll = TargetActorsWorld.Actors(i).Roll;
                    obj.ScenarioObj.Actors(i+1).Pitch = TargetActorsWorld.Actors(i).Pitch;
                    obj.ScenarioObj.Actors(i+1).Yaw = TargetActorsWorld.Actors(i).Yaw;
                    obj.ScenarioObj.Actors(i+1).AngularVelocity = TargetActorsWorld.Actors(i).AngularVelocity;
                end
               
                % Update plots
                updatePlots(obj.ScenarioObj);
                
                %% Update target list and predictions for plotting capsule list.
                obj.TargetIDs = double(futureTrajectory.TargetIDs(uint8(1):futureTrajectory.NumTrajs));
                obj.ActorGeom = repelem(obj.EgoGeom(1),futureTrajectory.NumTrajs,1);
                Lia = ismember(obj.CapList.ObstacleIDs,obj.TargetIDs);
                ReduceIDs = obj.CapList.ObstacleIDs(~Lia);
                if ~isempty(ReduceIDs)
                    removeObstacle(obj.CapList, ReduceIDs);
                end
                
                %% Update actor profiles using input ObstacleProfiles
                for i=1:numel(obj.TargetIDs)
                     for j = 1:numel(obj.TargetIDs)
                         % Get vehicles info from obstacle profiles for
                         % targets with valid index. In case of vehicle
                         % info from truth, index is always valid.
                         if(obj.TargetIDs(i)<= 0)
                             % In case of invalid traget index, use default
                             % vehicle info.
                             obj.ActorGeom(i).Geometry.Length = obj.CarLength;
                             obj.ActorGeom(i).Geometry.Width = obj.CarWidth;
                             obj.ActorGeom(i).Geometry.Radius = obj.CarWidth/2;
                         else
                             obj.ActorGeom(i).Geometry.Length = ObstacleProfiles(obj.TargetIDs(i)).Length;
                             obj.ActorGeom(i).Geometry.Width = ObstacleProfiles(obj.TargetIDs(i)).Width;
                             obj.ActorGeom(i).Geometry.Radius = ObstacleProfiles(obj.TargetIDs(i)).Width/2; 
                         end
                     end
                end
                updateObstacleGeometry(obj.CapList, obj.TargetIDs, obj.ActorGeom);
                updateObstacleGeometry(obj.CapListTopView, obj.TargetIDs, obj.ActorGeom);
                
                % Update obstacle poses.
                [~, obj.ActorPoses] = obstaclePose(obj.CapList, obj.CapList.ObstacleIDs);
                for i = uint8(1):futureTrajectory.NumTrajs
                    obj.ActorPoses(i).States = futureTrajectory.Trajectories(i).Trajectory(uint16(1):futureTrajectory.Trajectories(i).NumPts,1:3);
                end
                % Update the MIOs poses in the capsule list
                updateObstaclePose(obj.CapList, obj.TargetIDs, obj.ActorPoses);
                updateObstaclePose(obj.CapListTopView, obj.TargetIDs, obj.ActorPoses);
                
                % Update ego pose in to capsule list.
                obj.EgoPoses.States = GlobalTraj.GlobalTrajectory(OptimalTrajIdx).Trajectory(:,1:3);
                updateEgoPose(obj.CapList, obj.EgoActorID, obj.EgoPoses);
                updateEgoPose(obj.CapListTopView, obj.EgoActorID, obj.EgoPoses);
				
				% Set maximum number of trajectory points used for collision checking.
                obj.CapList.MaxNumSteps =size(GlobalTraj.GlobalTrajectory(OptimalTrajIdx).Trajectory,1);
                
                % Plot capsule list on chase view
                if obj.EnableChaseView && obj.EnableCaplistChaseView
                    hold(obj.HAxisChase,'on');
                    show(obj.CapList,'TimeStep',1:obj.CapList.MaxNumSteps,'FastUpdate',1,'Parent',obj.HAxisChase);
                    hold(obj.HAxisChase,'off');
                end 
                
                %% Update Velocity Info (Ego, Other Vehicle)
                if obj.EnableTopView && isgraphics(obj.EgoTextTformTop) && isgraphics(obj.EgoVelTextTop)
                    p   = EgoActor.Position;           % [x y z]
                    yaw = EgoActor.Yaw;                % deg
                    v   = EgoActor.Velocity;           % [vx vy vz]
                    spd = hypot(v(1), v(2)) * 3.6;     % km/h

                    % Ego의 월드 변환: 평면 회전(yaw) + 평행이동
                    T = makehgtform('translate', [p(1) p(2) p(3)], ...
                                    'zrotate' , deg2rad(yaw));

                    set(obj.EgoTextTformTop, 'Matrix', T);     
                    set(obj.EgoVelTextTop, 'String', sprintf('%.1f km/h', spd));
                end

                if obj.EnableTopView
                    numTgts = TargetActorsWorld.NumActors;

                    if numel(obj.VelTextHandlesTop) > numTgts
                        delete(obj.VelTextHandlesTop(numTgts+1:end));
                        obj.VelTextHandlesTop(numTgts+1:end) = [];
                    end

                    % 각 타깃에 대해 속도 텍스트 갱신
                    for i = 1:numTgts
                        tgt = TargetActorsWorld.Actors(i);

                        p   = tgt.Position;            % [x y z]
                        v   = tgt.Velocity;            % [vx vy vz]
                        yaw = tgt.Yaw;                 % (deg)

                        % 속도 [km/h]
                        spd_kmh = hypot(v(1), v(2)) * 3.6;
                        ahead   = (obj.CarLength * 0.5) + 1.5;      % meter
                        th      = deg2rad(yaw);
                        px      = p(1) + ahead * cos(th);
                        py      = p(2) + ahead * sin(th);
                        label = sprintf('%.1f km/h', spd_kmh);

                        % 핸들 생성/갱신
                        if numel(obj.VelTextHandlesTop) < i || ~isgraphics(obj.VelTextHandlesTop(i))
                            obj.VelTextHandlesTop(i) = text(obj.HAxisTop, px, py, label, ...
                                'Color', [1 0 0], ...
                                'FontWeight', 'bold', ...
                                'HorizontalAlignment', 'center', ...
                                'Tag', 'TargetSpeedText', ...
                                'Clipping', 'on');   % 축 밖으로 나가면 잘리도록
                        else
                            set(obj.VelTextHandlesTop(i), ...
                                'Position', [px, py, 0], ...
                                'String',  label, ...
                                'Visible', 'on');
                        end
                    end
                end
                
                % Disable closing of visualization window in between
                % simulation.
                if obj.EnableTopView || obj.EnableChaseView
                    % Display warning message dialog when user tries to close
                    % the window during simulation.
                    set(obj.Figure,'CloseRequestFcn',...
                        "warndlg({'Unable to close the Lane Change Status Plot during simulation. To disable Lane Change Status Plot, deselect ''Enable Chase view and Enable Top view checkboxes'''},'Warning');");
                end
                
                if obj.useGIF
                    % Capture the frame for the GIF
                    frame = getframe(obj.Figure);
                    img = frame2im(frame);
                    [imgInd, cm] = rgb2ind(img, 256);

                    % Write the frame to the GIF file
                    if obj.FrameCount == 0
                        imwrite(imgInd, cm, obj.GifFilename, 'gif', 'Loopcount', inf, 'DelayTime', 1/10);
                    else
                        imwrite(imgInd, cm, obj.GifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', 1/10);
                    end
                    obj.FrameCount = obj.FrameCount + 1;
                end

            end
        end
        
        function flag = isInactivePropertyImpl(obj,propertyName)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog

            switch propertyName
                case 'EnableTrajTopView'
                     flag = ~obj.EnableTopView;
                case 'EnableTrajChase'
                    flag = ~obj.EnableChaseView;
                case 'EnableCaplistChaseView'
                    flag = ~obj.EnableChaseView;
                otherwise
                    flag = false;
            end
        end

        function num = getNumInputsImpl(~)
            num = 4;
        end

        function varargout  = getInputNamesImpl(obj)
             numInputs = getNumInputs(obj);
             varargout = cell(1,numInputs);
             % Return input port names for System block
             varargout{1}  = 'TargetActorsWorld';
             varargout{2}  = 'VisualizationInfo';
             varargout{3}  = 'ObstacleProfiles';
             varargout{4}  = 'EgoActor';
        end
    end
    %----------------------------------------------------------------------
    % Simulink dialog
    %----------------------------------------------------------------------
    methods(Access = protected, Static)
        %------------------------------------------------------------------
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(....
                "Title","HelperPlotLCPlannerStatus",...
                "Text",...
                "Helps in visualizing the execution of trajectory using MATLAB figure in Chase view and Top view");
        end
        function groups = getPropertyGroupsImpl
            % Define property group for top view.
            topViewGroup = matlab.system.display.Section('Title','Top View parameters',...
                'PropertyList',{'EnableTopView','EnableTrajTopView'});
            % Define property group for chase view.
            chaseViewGroup = matlab.system.display.Section('Title','Chase View parameters',...
                'PropertyList',{'EnableChaseView','EnableTrajChase','EnableCaplistChaseView'});
            % Return an array with the two sections.
            groups = [topViewGroup, chaseViewGroup];
        end
         
        %------------------------------------------------------------------
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
        
        %------------------------------------------------------------------
        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
        
        %------------------------------------------------------------------
        function releaseImpl(obj)
            if obj.EnableChaseView || obj.EnableTopView
                % Modifying the CloseRequestFcn callback to its original state
                set(obj.Figure,'CloseRequestFcn','closereq');
            else
                close(obj.Figure);
            end
        end
    end
    
end