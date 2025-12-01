function [Target_vel_low, Target_vel_high] = helperCalculateTargetstates(EgoState, MIO_pos_s, MIO_vel, ...
                                                                         planner_params, feasible_coeff)

    % 다음과 같은 경우에서 Target Velocity를 지정
    %   1. 전방 MIO가 더 느리고, TTC가 [TTC_min, TTC_max] 범위 내에 잡히는 경우
    %   2. 후방 MIO가 더 빠르고, TTC가 [TTC_min, TTC_max] 범위 내에 잡히는 경우
    T     = planner_params.TimeHorizon;                                         
    v_max = planner_params.MaxLonVelocity;
    v_min = planner_params.MinLonVelocity;
    a_acc = planner_params.MaxLonAccel;
    a_brk = planner_params.MaxLonAccel;
    max_ttc = planner_params.MaxTTC;
    min_ttc = planner_params.MinTTC;
    MaxFront = planner_params.MaxFront; 
    MaxRear = planner_params.MaxRear;

    s0 = EgoState(1);
    v0 = EgoState(2);

    MIO_s0 = MIO_pos_s;
    MIO_v0 = MIO_vel;
    % Feasibility에 사용되는 Target Velocity위 범위.
    % 해당 범위 내에 존재하도록 Target 속도 범위를 지정해야 함.
    v_min_feasible = max(v_min, v0-a_brk*T*feasible_coeff);
    v_max_feasible = min(v_max, v0+a_acc*T*feasible_coeff);
    
    if s0 < MIO_pos_s
        % MIO가 차량 전방에 위치, v_MIO < v_ego
        % (0 -> 1), (v_ego -> v_min)
        ttc = max(min_ttc, (MIO_s0 - s0 - MaxFront) / (v0 - MIO_v0));
        Target_vel_low = v_min_feasible;
        Target_vel_high = (ttc-min_ttc) / (max_ttc-min_ttc) * (v0-v_min_feasible) + v_min_feasible;
    else
        % MIO가 차량 후방에 위치, v_MIO > v_ego
        % (0 -> 1), (v_ego -> v_max)
        ttc = max(min_ttc, (s0 - MIO_s0 - MaxRear) / (MIO_v0 - v0));
        Target_vel_low = (ttc-min_ttc) / (max_ttc-min_ttc) * (v0-v_max_feasible) + v_max_feasible;
        Target_vel_high = v_max_feasible;
    end
end