function validTrajectories = helperKinematicFeasibility(globalTraj,...
                                                        plannerParams)
%helperKinematicFeasibility function checks for kinematic feasibility of
%generated global trajectories.
%
% The helperKinematicFeasibility calculates maximum value of curvature,
% acceleration, yaw rate and minimum value of velocity for all
% the sampled trajectories. It validates the calculated values
% against user defined bounds.

% Get number of trajectories
numTrajectories = globalTraj.NumTrajectories;

% Initialize output variable
validTrajectories = globalTraj;

% Check every trajectory individually for different violations
for i = 1:numTrajectories
    % Get number of points in the trajectory
    numTrajPoints = globalTraj.GlobalTrajectory(i).NumTrajPoints;
    
    % Get the absolute value of curvature
    curvature = abs(globalTraj.GlobalTrajectory(i).Trajectory(1:numTrajPoints,4));
    
    % Get the velocity of trajectory
    velocity = globalTraj.GlobalTrajectory(i).Trajectory(1:numTrajPoints,5);
    
    % Compute yawrate.
    yawrate = curvature.*abs(velocity);
    
    % Find the maximum values of curvature, yawrate and
    % acceleration
    maxAccel = max(abs(globalTraj.GlobalTrajectory(i).Trajectory(1:numTrajPoints,6)));
    maxCurv = max(curvature);
    maxYawrate = max(yawrate);
    
    % Evaluate for acceleration constraint
    accViolated  = maxAccel > abs(plannerParams.MaxAccel);
    
    % Evaluate for curvature constraint
    curvViolated = maxCurv > abs(plannerParams.MaxCurvature);
    
    % Evaluate for yawrate constraint
    yawrateViolated = maxYawrate > abs(plannerParams.MaxYawRate);
    
    % Evaluate for velocity constraint
    velViolated  = any(velocity < plannerParams.MinVelocity);
    
    % Update the validity of the trajectory
    isValid = ~accViolated && ~curvViolated && ~velViolated && ~yawrateViolated;
    validTrajectories.GlobalTrajectory(i).IsValid = isValid;
end