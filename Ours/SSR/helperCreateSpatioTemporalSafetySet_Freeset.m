function SpatioTemporalSafetySet = helperCreateSpatioTemporalSafetySet_Freeset(mioRelativeFrenetStates, ...
                                                                               mioGlobalFrenetStates, ...
                                                                               MapInfo, ...
                                                                               initStruct, ...
                                                                               EgoState, ...
                                                                               EgoActor, ... 
                                                                               EgoLane, ...
                                                                               planner_params)
%%% mioRelativeFrenetStates %%%
%%% 1. NumMIOs      : MIO 개수
%%% 2. TargetIds    : MIO 차량들의 고유 번호
%%% 3. MIOStates    : MIO 차량들의 Frenet States w.r.t Relative (s ds dds l dl ddl)

%%% mioGlobalFrenetStates %%%
%%% NumMIOs     : MIO 개수
%%% TargetIds   : MIO 차량들의 고유 번호
%%% MIOStates   : MIO 차량들의 Frenet States w.r.t Global (s ds dds l dl ddl)

%%% FutureGlobalTrajectory %%%
%%% 1. NumTrajs     : Trajectroy 개수 (= MIO 개수)
%%% 2. TargetIDs    : MIO 고유 번호
%%% 3. Trajectories : Trajectory 객체 (TargetID, NumPts, Trajectory)

%%% MapInfo %%%
%%% 1. NumLanes             : 차선 개수
%%% 2. LaneWidth            : 차선 폭
%%% 3. LaneCenter           : 차선 중심
%%% 4. NumGlobalPlanPoints  : Global Waypoint 개수
%%% 5. GlobalPlanPoints     : Global Waypoint 초기경로
%%% 6. LaneLateralBoundary  : 차선의 횡방향 Boundary

%%% Actors %%%
%%% 1. ActorID          : MIO 차량 고유 번호
%%% 2. Length           : MIO 차량 고유 전장
%%% 3. width            : MIO 차량 고유 전폭

N_t = 9; % [0, 0.5, ... 4.0]
N_l = 3; % [Right Lane, Ego Lane, Left Lane]
N_k = 5; % 한 차선 MIO 최대 2개 + 옆 차선 MIO 최대 2개 -> 공간 최대 5개
N_R = 5*2; % region 범위 5개 x 2 = 10개
planning_horizon = planner_params.TimeHorizon;
planning_resolution = planner_params.PlanningResolution;
v_max = planner_params.MaxLonVelocity;
v_min = planner_params.MinLonVelocity;
a_max = planner_params.MaxLonAccel;
a_min = -planner_params.MaxLonAccel;
ttc_max = planner_params.MaxTTC;
maxFront = planner_params.MaxFront;
maxRear = planner_params.MaxRear;
feasible_coeff = planner_params.Feasible_coeff;

LaneWidth = MapInfo.LaneWidth(1);
LaneCenters = MapInfo.LaneCenters;

SpatioTemporalSafetySet = initStruct;

MIOIds = mioRelativeFrenetStates.TargetIds;
NumMIOs = mioRelativeFrenetStates.NumMIOs;

egolane = EgoLane;
ValidLanes_G = egolane + [1, 0, -1];
ValidLanes = egolane + [1, 0, -1];
ValidLanes_G(ValidLanes_G < 1 | ValidLanes_G > MapInfo.NumLanes) = [];
ValidLanes(ValidLanes < 1 | ValidLanes > MapInfo.NumLanes) = [];
ValidLanes = ValidLanes - egolane; % 상대 좌표로 변환
NumLanes = length(ValidLanes);
front_MIO_States = zeros(NumLanes, 6);
rear_MIO_States = zeros(NumLanes, 6);

Boundaries_Lon = zeros(N_t, NumLanes, N_R); % 최대 영역 5개 x 2 = 10
Boundaries_Lat = repmat(struct('MinL', 0, 'MaxL', 0), NumLanes, 1);
for i = 1:NumLanes
    % Ego 기준 오, 중앙, 왼 순으로 저장
    r = ValidLanes(i);                % r ∈ {...,1,0,-1,...}
    center = -r * LaneWidth;           % Ego 차선 중심이 0 → 옆 차선은 ±LaneWidth, ...
    Boundaries_Lat(i).MinL = center - LaneWidth/2;
    Boundaries_Lat(i).MaxL = center + LaneWidth/2;
    [front_MIO_State, rear_MIO_State] = select_target_mio(center, ...
                                                          EgoState, ...
                                                          LaneCenters(egolane), ...
                                                          mioGlobalFrenetStates, ...
                                                          MapInfo);
    front_MIO_States(i, :) = front_MIO_State;
    rear_MIO_States(i, :) = rear_MIO_State;
end
time_interval = 0 : planning_resolution : planning_horizon;

% 1. Boundary B(t) 생성
% 2. Gaps G(t) 생성
% 3. Freeset F(t) 생성
%% Occupied Region 정의
ego_offset = EgoState(4) - LaneCenters(egolane);
occupied_regions = zeros(N_t, N_l, N_k, 2);
num_regions = zeros(N_t, N_l);
veh_length = EgoActor.Length;
veh_width = EgoActor.Width;
for i = 1:numel(time_interval)-1
    % 모든 Time Interval에 대해 반복
    t1 = time_interval(i);
    t2 = time_interval(i+1);
    for j = 1:NumLanes
        % 모든 유효 Lane에 대해 반복
        m = 0;
        lane_upper = Boundaries_Lat(j).MaxL;
        lane_lower = Boundaries_Lat(j).MinL;
        for k = 1:NumMIOs
            % [s, ds, dds, l, dl, ddl]
            % 모든 MIO에 대해 반복
            MIO_init_rel_state = mioRelativeFrenetStates.MIOStates(k, :);
            MIO_init_glo_state = mioGlobalFrenetStates.MIOStates(k, :);

            s_r = MIO_init_rel_state(1);
            l_r = MIO_init_rel_state(4) + ego_offset;
            vs_g = MIO_init_glo_state(2);
            vl_g = MIO_init_glo_state(5);

            d_min_t1 = l_r + vl_g*t1 - veh_width/2;
            d_max_t1 = l_r + vl_g*t1 + veh_width/2;
            d_min_t2 = l_r + vl_g*t2 - veh_width/2;
            d_max_t2 = l_r + vl_g*t2 + veh_width/2;
            d_vehicle_min = min([d_min_t1, d_min_t2]);
            d_vehicle_max = max([d_max_t1, d_max_t2]);

            if (d_vehicle_min <= lane_upper) && (d_vehicle_max >= lane_lower)
                % 시간 t에서 특정 MIO가 해당 Lane에 속한 경우
                s_min_t1 = s_r + vs_g*t1 - veh_length/2;
                s_max_t1 = s_r + vs_g*t1 + veh_length/2;
                s_min_t2 = s_r + vs_g*t2 - veh_length/2;
                s_max_t2 = s_r + vs_g*t2 + veh_length/2;
                                        
                s_occupied_min = min([s_min_t1, s_min_t2]);
                s_occupied_max = max([s_max_t1, s_max_t2]);
                
                % 4D 배열에 추가
                m = m+1;
                occupied_regions(i, j, m, :) = [s_occupied_min, s_occupied_max];
            end
        end
        num_regions(i, j) = m;
    end
end

%% Boundary Set B(t) 정의
num_bounds = zeros(N_t, NumLanes);
for i = 1:N_t
    for j = 1:NumLanes
        c_idx = 1;
        num_region = num_regions(i, j);
        if num_region > 0
            % (N_r, 2)
            region = squeeze(occupied_regions(i, j, 1:num_region, :));
            if num_region == 1
                region = region';
            end
            sorted_regions = sortrows(region, 1);
            % 겹치는 영역들을 병합
            total_num = 0;
            merged_regions = zeros(num_region, 2);
            current_region = sorted_regions(1, :);
            for k = 2:num_region
                next_region = sorted_regions(k, :);
                if current_region(2) >= next_region(1)
                    current_region(2) = max(current_region(2), next_region(2));
                else
                    % 병합된 최종 region의 개수를 카운팅
                    total_num = total_num + 1;
                    merged_regions(k-1, :) = current_region;
                    current_region = next_region;
                end
            end
            total_num = total_num + 1;
            merged_regions(total_num, :) = current_region;

            % 첫 번째 점유 영역 이전 자유 구간
            if merged_regions(1, 1) > -150
                Boundaries_Lon(i, j, c_idx:c_idx+1) = [-150, merged_regions(1,1)];
                c_idx = c_idx + 2;
            end

            % 점유 영역들 사이의 자유 구간
            for m = 1:total_num-1
                gap_start = merged_regions(m, 2);
                gap_end = merged_regions(m+1, 1);
                if gap_end > gap_start
                    Boundaries_Lon(i, j, c_idx:c_idx+1) = [gap_start, gap_end];
                    c_idx = c_idx + 2;
                end
            end

            % 마지막 점유 영역 이후 자유 구간
            if merged_regions(total_num, 2) < 150
                Boundaries_Lon(i, j, c_idx:c_idx+1) = [merged_regions(total_num, 2), 150];
                c_idx = c_idx + 2;
            end
        else
            Boundaries_Lon(i, j, 1:2) = [-150, 150];
            c_idx = c_idx + 2;
        end
        num_bounds(i, j) = (c_idx-1)/2;
    end
end

%% Lane 별 Terminal Velocity Range 생성
terminal_velocity_range = zeros(3, 2);
for i = 1:NumLanes
    r = ValidLanes(i);
    % ego-centric row로 변환: 1=Right, 2=Ego, 3=Left
    row = 2 - sign(r);

    % 차선별 계산값 리셋
    Target_vel_low  = 0;
    Target_vel_high = 0;

    front_mio_state = front_MIO_States(i, :);
    rear_mio_state  = rear_MIO_States(i, :);

    is_front = ~all(front_mio_state == 0);
    is_rear  = ~all(rear_mio_state  == 0);

    if is_front && is_rear
        if abs(front_mio_state(1) - EgoState(1)) < abs(rear_mio_state(1) - EgoState(1))
            target_mio_state = front_mio_state;
        else
            target_mio_state = rear_mio_state;
        end
    elseif is_front
        target_mio_state = front_mio_state;
    elseif is_rear
        target_mio_state = rear_mio_state;
    else
        target_mio_state = zeros(1,6);
    end

    MIO_pos_s = target_mio_state(1);
    MIO_vel   = target_mio_state(2);

    if MIO_pos_s == 0 && MIO_vel == 0
        % 0. MIO 없는 경우
        Target_vel_low = min(EgoState(2)+a_max*planning_horizon*feasible_coeff, v_max*feasible_coeff);
        Target_vel_high = v_max;
    else
        % 1. 앞 MIO가 빠른 경우
        if (MIO_pos_s - EgoState(1) > 0 && MIO_vel > EgoState(2))
            Target_vel_low  = min(EgoState(2)+a_max*planning_horizon*feasible_coeff, MIO_vel);
            Target_vel_low  = min(Target_vel_low, v_max*feasible_coeff);
            Target_vel_high = v_max;
        end

        % 2. 뒤 MIO가 느린 경우
        if (MIO_pos_s - EgoState(1) < 0 && MIO_vel < EgoState(2))
            Target_vel_low  = min(EgoState(2)+a_max*planning_horizon*feasible_coeff, v_max*feasible_coeff);
            Target_vel_high = v_max;
        end

        % 3. 앞 MIO가 느리지만, 최대 TTC 바깥인 경우
        if (MIO_pos_s - EgoState(1) > 0 && MIO_vel < EgoState(2))
            ttc = (MIO_pos_s - EgoState(1) - maxFront) / (EgoState(2) - MIO_vel);
            if ttc >= ttc_max
                Target_vel_low  = EgoState(2);
                Target_vel_high = v_max;
            end
        end

        % 4. 뒤 MIO가 빠르지만, 최대 TTC 바깥인 경우
        if (MIO_pos_s - EgoState(1) < 0 && MIO_vel > EgoState(2))
            ttc = (EgoState(1) - MIO_pos_s - maxRear) / (MIO_vel - EgoState(2));
            if ttc >= ttc_max
                Target_vel_low  = min(EgoState(2)+a_max*planning_horizon*feasible_coeff, MIO_vel);
                Target_vel_low  = min(Target_vel_low, v_max*feasible_coeff);
                Target_vel_high = v_max;
            end
        end

        % fallback (target band가 아직 안 정해진 경우)
        if Target_vel_low == 0 || Target_vel_high == 0
            [Target_vel_low, Target_vel_high] = helperCalculateTargetstates( ...
                EgoState, MIO_pos_s, MIO_vel, planner_params, feasible_coeff);
        end
    end
    terminal_velocity_range(row, :) = [Target_vel_low, Target_vel_high];
end


%% Gap G(t) & Free set F(t) 정의
gap_ids = 1;
for i = 1:N_t-1
    f_t = HelperLCPlannerDefaultData.FreeSet;
    time_local_ids = 1;
    for j = 1:NumLanes
        % 특정 Lane, 특정 Time에서의 Boundary b(t)
        front_mio_state = front_MIO_States(j, :);
        rear_mio_state = rear_MIO_States(j, :);
        is_front = ~all(front_mio_state == 0);
        is_rear = ~all(rear_mio_state == 0);
        b_t = squeeze(Boundaries_Lon(i, j, :));
        num_bound = num_bounds(i, j);
        f_t.num_gaps_lane(j) = num_bound;
        for k = 1:num_bound
            % Boundary Clipping
            % 1) min, max Clipping
            b_t_rear = max(-150, min(150, b_t(2*k-1)));
            b_t_front = max(-150, min(150, b_t(2*k)));
            % g(t) 특정 lane, 특정 time에서의 단일 Gap -> 집합으로 확장
            gap = HelperLCPlannerDefaultData.LaneChangeGap;
            gap.idx = gap_ids;
            gap.time_local_idx = time_local_ids;
            gap.lane_idx = j;
            gap.global_lane_idx = ValidLanes_G(j); % Lane의 Global Index
            gap.time_idx = i;
            % 2) TODO: Ego Vehicle Clipping
            gap.s = (b_t_rear + b_t_front) / 2;
            gap.d = (Boundaries_Lat(j).MinL + Boundaries_Lat(j).MaxL) / 2; % Ego Centric Coordinate
            gap.l = b_t_front - b_t_rear;
            gap.w = LaneWidth;
            gap.IsValid = true;
            
            % 3) Node Cost Calculation
            % 각 gap을 기준으로 front & rear 어디에 위치하는지 식별
            [is_gap_front, is_gap_rear, ...
             gap_front_mio_state, gap_rear_mio_state] = select_target_mio_gap(gap, ...
                                                                              is_front, ...
                                                                              is_rear, ...
                                                                              front_mio_state, ...
                                                                              rear_mio_state, ...
                                                                              EgoState);
                
            % 선별된 front & rear mio를 바탕으로 distance weight 계산
            front_mio_s = gap_front_mio_state(1);
            rear_mio_s = gap_rear_mio_state(1);
            distance_weight_f = double(is_gap_front) * ...
                                (1 - abs(front_mio_s-EgoState(1)) / 150); 
            distance_weight_r = double(is_gap_rear) * ...
                                (1 - abs(rear_mio_s-EgoState(1)) / 150);

            % distance weighted transportation cost 계산
            front_mio_v = gap_front_mio_state(2);
            rear_mio_v = gap_rear_mio_state(2);
            transport_cost = 1.0 * (distance_weight_f * (1-front_mio_v/v_max) + ... 
                                    distance_weight_r * rear_mio_v/v_max);
            
            % Front length cost 계산
            s_max = max(0, gap.s + gap.l/2); 
            s_min = max(0, gap.s - gap.l/2);
            front_length_cost = 2.0 * max(0, (1 - (s_max - s_min) / 150));
            
            gap.node_cost = transport_cost + front_length_cost;
            
            % 4) Time-interval velocity range
            v_low = max(v_min, terminal_velocity_range(:, 1) - a_max * (N_t-1 - i) * planning_resolution);
            v_high = min(v_max, terminal_velocity_range(:, 2) - a_min * (N_t-1 - i) * planning_resolution);

            gap.vel_range = [v_low, v_high];


            % f(t) 특정 time에서 모든 Lane에 대한 Gap 집합 생성
            f_t.G_t(j, k) = gap;

            gap_ids = gap_ids + 1;
            time_local_ids = time_local_ids + 1;
        end
    end
    f_t.num_gaps = sum(f_t.num_gaps_lane);
    f_t.time_idx = i;

    % F(t) 모든 time, 모든 lane에 대한 Gap 집합
    SpatioTemporalSafetySet.F_t(i) = f_t;
end

end


function [front_MIO_State, rear_MIO_State] = select_target_mio(Target_d, EgoState, EgoLane_d, MIOState, MapInfo)
    Ego_pos_s = EgoState(1);
    NumMIOs = MIOState.NumMIOs;
    MIOStates = MIOState.MIOStates;
    LaneWidth = MapInfo.LaneWidth(1);
    front_MIO_State = zeros(1, 6);
    rear_MIO_State = zeros(1, 6);
    % Target으로 잡아야 하는 MIO 선정
    if NumMIOs > 0
        MIO_d_e = MIOStates(1:NumMIOs, 4) - (Target_d + EgoLane_d);
        valid_MIO_ids = find(abs(MIO_d_e) <= LaneWidth/2);
        if ~isempty(valid_MIO_ids)
            % Valid MIO가 존재하는 경우
            valid_MIOState = MIOStates(valid_MIO_ids, :); % (n, 6)
            if size(valid_MIOState, 1) > 1
                % Valid MIO가 2대인 경우
                front = (valid_MIOState(:, 1) - EgoState(1) > 0);
                rear = ~front;
                front_MIO_State(:) = valid_MIOState(front, :);
                rear_MIO_State(:) = valid_MIOState(rear, :);
            else
                % Valid MIO가 1대인 경우
                target_MIOState = valid_MIOState(1, :);
                MIO_pos_s = target_MIOState(1); % s
                if MIO_pos_s > Ego_pos_s
                    % 전방
                    front_MIO_State = target_MIOState;
                else
                    % 후방
                    rear_MIO_State = target_MIOState;
                end
            end
        end
    end
end


function [is_gap_front, is_gap_rear, ...
          gap_front_mio_state, gap_rear_mio_state] = select_target_mio_gap(gap, ...
                                                                           is_front, is_rear, ...
                                                                           front_mio_state, rear_mio_state, ...
                                                                           EgoState)
    gap_front_mio_state = zeros(1, 6);
    gap_rear_mio_state  = zeros(1, 6);
    is_gap_front = false;
    is_gap_rear = false;

    if is_front && is_rear
        front_mio_s = front_mio_state(1);
        rear_mio_s = rear_mio_state(1);
        delta_s = [front_mio_s, rear_mio_s] - EgoState(1);
        if all(gap.s < delta_s)
            % 둘 다 gap front 위치
            % 더 가까운 차량으로 기준 설정
            gap_front_mio_state(:) = rear_mio_state;
            is_gap_front = true;
        elseif all(gap.s > delta_s)
            % 둘 다 gap rear 위치
            % 더 가까운 차량으로 기준 설정
            gap_rear_mio_state(:) = front_mio_state;
            is_gap_rear = true;
        else
            % 양쪽으로 위치
            gap_front_mio_state(:) = front_mio_state;
            gap_rear_mio_state(:) = rear_mio_state;
            is_gap_front = true;
            is_gap_rear = true;
        end

    elseif is_front
        front_mio_s = front_mio_state(1);
        if gap.s > (front_mio_s - EgoState(1))
            % 해당 gap 기준 front mio는 뒤쪽에 생성
            gap_rear_mio_state(:) = front_mio_state;
            is_gap_rear = true;
        else
            % 해당 gap 기준 front mio는 앞쪽에 생성
            gap_front_mio_state(:) = front_mio_state;
            is_gap_front = true;
        end

    elseif is_rear
        rear_mio_s = rear_mio_state(1);
        if gap.s > (rear_mio_s - EgoState(1))
            % 해당 gap 기준 rear mio는 뒤쪽에 생성
            gap_rear_mio_state(:) = rear_mio_state;
            is_gap_rear =  true;
        else
            % 해당 gap 기준 rear mio는 앞쪽에 생성
            gap_front_mio_state(:) = rear_mio_state;
            is_gap_front = true;
        end
    end
end

