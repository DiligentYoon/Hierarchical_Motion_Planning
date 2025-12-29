function [EndOfPlan,RefPoseCurr,RefCurvature,RefVelocity,RefAcc,...
          AngVel, RefCurvaDer, LatOffset,RefYawDiff,...
          sectionStartIndex] = helperComputeCurrentStateBasedOnPose(XYPos,...
          NewPlan, waypoints, RefCurvatures, RefSpeedProfile,...
          RefAcceleration, RefAngVel, RefCurvaDerv, RefPoseMinus,...
          RefCurvatureMinus, RefVelocityMinus, RefAccMinus,...
          sectionStartIndex)
%helperComputeCurrentStateBasedOnPose computes the next best point from the
%trajectory based on current pose of the ego vehicle.
%
% This function calculates the next best point from the trajectory
% The output of this block will be used to calculate the inputs for path
% following controller.
%
%   This is a helper function for example purposes and
%   may be removed or modified in the future.
%
%   Copyright 2020 The MathWorks, Inc.

if NewPlan
    sectionStartIndex = 1;
end

if sectionStartIndex < 1
    sectionStartIndex = 1;
end

EndOfPlan    = false;
RefPose      = zeros(1, 3);
RefPoseCurr  = zeros(1, 3);
RefCurvature = 0;
RefVelocity  = 1;


numWaypoints = size(waypoints,1);
if (numWaypoints == 0)
    return;
end

% Distance between section start and end points
DeltaXY = [waypoints(sectionStartIndex+1,1)-waypoints(sectionStartIndex,1),...
           waypoints(sectionStartIndex+1,2)-waypoints(sectionStartIndex,2)];

% Distance between current position and section starting point
RXY = [XYPos(1)-waypoints(sectionStartIndex,1),...
       XYPos(2)-waypoints(sectionStartIndex,2)];

% Normalized distance between current position and section starting point
u = (RXY.*DeltaXY)/(DeltaXY.*DeltaXY);

% Find section ending point
indexIncrement = ceil(u-1);

if indexIncrement<0
    % In current section
    indexIncrement = 0;
end
if u >=1
    % Increment to appropriate section 
    % with the assumption that the distance between waypoints is 
    % approximately equal for near sections
    sectionStartIndex = sectionStartIndex+indexIncrement;
    
    % Adjust u to account for new section starting point
    u = u - indexIncrement;
end

if sectionStartIndex < (numWaypoints-1)
    % Operating within valid sections
    currentSectionEndIndex = sectionStartIndex+1;
    nextSectionEndIndex    = currentSectionEndIndex+1;
    % Target States at end point of current and next sections
    XYTarget0 = [waypoints(sectionStartIndex,1),...
                 waypoints(sectionStartIndex,2)];
    XYTarget1 = [waypoints(currentSectionEndIndex,1),...
                 waypoints(currentSectionEndIndex,2)];
    XYTarget2 = [waypoints(nextSectionEndIndex,1),...
                 waypoints(nextSectionEndIndex,2)];
    YawTarget0 = waypoints(sectionStartIndex,3);
    YawTarget1 = waypoints(currentSectionEndIndex,3);
    YawTarget2 = waypoints(nextSectionEndIndex,3);
    
    VelocityTarget1 = RefSpeedProfile(currentSectionEndIndex);
    VelocityTarget2 = RefSpeedProfile(nextSectionEndIndex);
    
    AccTarget1 = RefAcceleration(currentSectionEndIndex);
    AccTarget2 = RefAcceleration(nextSectionEndIndex);
    
    CurvatureTarget1 = RefCurvatures(currentSectionEndIndex);
    CurvatureTarget2 = RefCurvatures(nextSectionEndIndex);
    
    CurvaDerTarget1 = RefCurvaDerv(currentSectionEndIndex);
    CurvaDerTarget2 = RefCurvaDerv(nextSectionEndIndex);
    
    RefAngVel1 = RefAngVel(currentSectionEndIndex);
    RefAngVel2 = RefAngVel(nextSectionEndIndex);
    
    % Section weights         
    Weight1 = (1-u);
    Weight2 = u;
    % Target position
    XYTargetCurr = Weight1*XYTarget0+Weight2*XYTarget1;
    YawTargetCurr = Weight1*YawTarget0+Weight2*YawTarget1;
    XYTarget = Weight1*XYTarget1+Weight2*XYTarget2;
    YawTarget = Weight1*YawTarget1+Weight2*YawTarget2;
    % Desired RefPose
    RefPoseCurr = [XYTargetCurr rad2deg(YawTargetCurr)];
    RefPose = [XYTarget rad2deg(YawTarget)];
    % Desired velocity
    RefVelocity = Weight1*VelocityTarget1+Weight2*VelocityTarget2;
    
    %Desired Acceleration
    RefAcc = Weight1*AccTarget1 + Weight2*AccTarget2;
    
    % Desired curvature
    RefCurvature = Weight1*CurvatureTarget1+Weight2*CurvatureTarget2;
    RefCurvaDer = Weight1*CurvaDerTarget1+Weight2*CurvaDerTarget2;
    EndOfPlan = false;
    
    omega = Weight1*RefAngVel1+Weight2*RefAngVel2;
    
    AngVel = omega;
else
    % Maintain last heading between points
    RefPoseCurr = RefPoseMinus;   
    RefVelocity = RefVelocityMinus;
    RefCurvature = RefCurvatureMinus;
    RefCurvaDer  = 0;
    sectionStartIndex = numWaypoints-1;
    EndOfPlan = true;
    RefAcc = RefAccMinus;
    AngVel = 0;
end

% Distance between current position and RefPosCurr
Dist2Ref = [XYPos(1)-RefPoseCurr(1),...
       XYPos(2)-RefPoseCurr(2)];

DirSign = sign(CrossProd(DeltaXY(1),DeltaXY(2),Dist2Ref(1),Dist2Ref(2)));
LatOffset = -sqrt(Dist2Ref(1)^2+Dist2Ref(2)^2)*DirSign;
RefYawDiff = RefPoseCurr(3)-XYPos(3);
end

%%CrossProd Computes cross product of two vectors.
function CP = CrossProd(x1,y1,x2,y2)
CP = x1*y2-y1*x2;
end
