classdef BehaviorType < uint8
%BehaviorType class defines enumeration for different behaviors supported
%by lane change planner
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%    
% Copyright 2020 The MathWorks, Inc.
   enumeration
      CruiseControl(1),
      LeadCarFollowing(2),
      LaneChange(3)
   end
end