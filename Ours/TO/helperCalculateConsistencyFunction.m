function Consistency_Cost = helperCalculateConsistencyFunction(ego_d, prev_d, target_d)

    % Discussion : Upper Limit, Constant Penalty of Revision
    ConstantPenalty = 0.1;

    lat_ratio = abs(target_d - ego_d) / (abs(prev_d - ego_d) + 1e-6);

    Consistency_Cost = ConstantPenalty + clip(lat_ratio-1, 0, 1);
end
