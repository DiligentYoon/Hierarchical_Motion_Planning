function [RangeMag,RangeAng,VelMag,ttc,front] = helperCalculateTTC(egoFrenetState,mioFrenetState)
%helperCalculateTTC calculate time-to-contact between ego and MIO
% 
% This function computes TTC between ego and MIO. It also computes range
% magnitude, range angle, velocity magnitude and boolean variable to
% indicate front MIO.
%
%   This is a helper function for example purposes and
%   may be removed or modified in the future.
%
%   Copyright 2020 The MathWorks, Inc.

% Get relative long distance
relLongDist = mioFrenetState(1) - egoFrenetState(1); 

% Check if mio is located in front of ego
if relLongDist>0
    front = true;
else
    front = false;
end

relLatDist  = mioFrenetState(4) - egoFrenetState(4); % relative lat distance
relLongVel  = mioFrenetState(2) - egoFrenetState(2); % relative long velocity
relLatVel   = mioFrenetState(5) - egoFrenetState(5); % relative lat velocity
    
[RangeAng,RangeMag] = cart2pol(relLongDist,relLatDist); % relative range
[VelAng,vel] = cart2pol(relLongVel,relLatVel);
VelMag = vel * cos(VelAng-RangeAng); % relative velocity

% limit speed to +/-0.05 m/s for ttc calculation
if abs(VelMag)<0.05
    if VelMag>=0
        vel(:) = 0.05;
    else
        vel(:) = -0.05;
    end
else
    vel(:) = VelMag;
end

% Time-To-Contact (sec)
ttc = RangeMag/vel;

end