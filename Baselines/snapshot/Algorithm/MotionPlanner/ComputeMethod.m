classdef ComputeMethod < uint8
%ComputeMethod class defines enumeration for different compute methods used
%to compute the reference point on path in motion planner model.
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%    
% Copyright 2021 The MathWorks, Inc.
   enumeration
      TimeBased(1),
      PoseBased(2)     
   end
end