function [RePoseRr, Curvature,...
          Velocity, Acceleration,....
          AngVel, CurvaDer,....
          LatOffset,RefYawDiff,...
          replanFlag] = helperComputeCurrentStateBasedOnTime(clock,...
                                                           replanTime,...
                                                           numTrajPoints,...
                                                           trajOutCart,....
                                                           DurationTime,...
                                                           times,...
                                                           timeResolution)
%helperComputeCurrentStateBasedOnTime computes the next best point from
%the trajectory based on time.
%
% This function calculates the next best point from the trajectory based on
% time. The output of this block will be used to calculate the inputs for
% pack ego block.
%
%   This is a helper function for example purposes and
%   may be removed or modified in the future.
%
%   Copyright 2020 The MathWorks, Inc.

% Get time delta
timeDelta = clock-replanTime+timeResolution;
% Get time vector
timeVecOut = times(1:numTrajPoints);

if timeDelta > timeVecOut(end)
    timeDelta = timeVecOut(end);
end
% Compute Omega
Omega = gradient(trajOutCart(1:numTrajPoints,3),timeVecOut);

% Compute position
PositionX    = interp1(timeVecOut(1:numTrajPoints),trajOutCart(1:numTrajPoints,1),timeDelta);
PositionY    = interp1(timeVecOut(1:numTrajPoints),trajOutCart(1:numTrajPoints,2),timeDelta);

% Compute orientation
Yaw          = interp1(timeVecOut(1:numTrajPoints),trajOutCart(1:numTrajPoints,3),timeDelta);
RePoseRr     = [PositionX PositionY rad2deg(Yaw)];

% Compute curvature
Curvature    = interp1(timeVecOut(1:numTrajPoints),trajOutCart(1:numTrajPoints,4),timeDelta);

% Compute velocity
Velocity     = interp1(timeVecOut(1:numTrajPoints),trajOutCart(1:numTrajPoints,5),timeDelta);

% Compute acceleration
Acceleration = interp1(timeVecOut(1:numTrajPoints),trajOutCart(1:numTrajPoints,6),timeDelta);

% Compute Angular velocity
AngVel        = interp1(timeVecOut(1:numTrajPoints),Omega(1:numTrajPoints),timeDelta);

% Compute curvature derivative
CurvaDerState = gradient(trajOutCart(1:numTrajPoints,4))./gradient(timeVecOut)./trajOutCart(1:numTrajPoints,5);
CurvaDer     = interp1(timeVecOut(1:numTrajPoints),CurvaDerState,timeDelta);

LatOffset = 0;
RefYawDiff = 0;
% replanFlag = (timeDelta >= (DurationTime - 2*timeResolution));
replanFlag = timeDelta >= DurationTime;
end