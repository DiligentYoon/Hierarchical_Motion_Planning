function Gaps = helperEvaluateTrafficCost(numGaps, Gaps, egoInfo, MapInfo, mio_info, Num_veh, Ids_veh)

%%% Gaps     : BusLaneChangeGaps 
%%% MapInfo  : BusMapInfo
%%% Num_veh  : Number of mios in each lanes
%%% Ids_veh  : mio's index
%%% mio_info : miostates (1행 : s, 2행 : global_velocity, 3행 : lateraldeviation, 3행 : mioids)
%%% ego_info : egostates (1행 : s, 2행 : global_velocity, 3행 : lateraldeviation) 

%%% 차선에 대한 정보 + 각 차선에 존재하는 차량 대수
%%% 각 차선에 존재하는 차량들의 절대속도, ego 차량의 절대속도

ego_s = double(egoInfo(1));
ego_vel = double(egoInfo(2));
numLanes = MapInfo.NumLanes;    
total_car = sum(Num_veh(1:numLanes));
mio_lane = zeros(2,total_car);
mio_lane(1, :) = mio_info(4, :);
mio_lane(2, :) = mio_info(5, :);

for i = 1:numGaps
    lane_number = Gaps(i).lane_idx;
    if Num_veh(lane_number) == 0
        Gaps(i).traffic_cost = 0;
    else
        n_car = Num_veh(lane_number);
        C_tra_1 = 0.1 * n_car / total_car;
        mioInLaneidx = find(mio_lane(2,:) == lane_number);
        mio_Ids = mio_lane(1, mioInLaneidx);

        if length(mio_Ids) == 1
            target_idx = find(mio_info(4,:) == mio_Ids);
            surround_x = double(mio_info(1, target_idx)) - ego_s;
            surround_vel = double(mio_info(2, target_idx));
            coeff = (50 - abs(surround_x)) / 50;

            if Gaps(i).s < surround_x
                C_tra_2 = coeff * abs(ego_vel) / surround_vel;
            else
                C_tra_2 = coeff * (surround_vel / ego_vel);
            end

        else
            target_idx = ismember(mio_info(4,:), mio_Ids);
            surround_x = double(mio_info(1, target_idx)) - ego_s;
            surround_vel = double(mio_info(2, target_idx));
            front_surround_idx = surround_x > Gaps(i).s;
            front_surround_x = surround_x(front_surround_idx);
            front_surround_vel = surround_vel(front_surround_idx);
            num_front = length(front_surround_x);

            rear_surround_idx = surround_x < Gaps(i).s;
            rear_surround_x = surround_x(rear_surround_idx);
            rear_surround_vel = surround_vel(rear_surround_idx);
            num_rear = length(rear_surround_x);

            if num_front > 1 && num_rear == 0
                [target_x, min_idx] = min(front_surround_x);
                target_vel = front_surround_vel(min_idx);
                coeff = (50 - abs(target_x)) / 50;
                C_tra_2 = coeff * abs(target_vel - ego_vel) / target_vel;
                
            elseif num_front == 0 && num_rear > 1
                [target_x, max_idx] = max(rear_surround_x);
                target_vel = rear_surround_vel(max_idx);
                coeff = (50 - abs(target_x)) / 50;
                C_tra_2 = coeff * abs(target_vel) / ego_vel;
            else
                [front_target, front_min_idx] = min(front_surround_x);
                front_surround_vel = front_surround_vel(front_min_idx);
                [rear_target, rear_max_idx] = max(rear_surround_x);
                rear_surround_vel = rear_surround_vel(rear_max_idx);
                deltaego = abs(ego_s - [front_target, rear_target]);
                if deltaego(1) < deltaego(2)
                    target_x = front_target;
                else
                    target_x = rear_target;
                end
                coeff = (50 - abs(target_x)) / 50;
                C_tra_2 = coeff * 0.5 * ((rear_surround_vel / ego_vel) + (abs(front_surround_vel - ego_vel) / front_surround_vel));
            end


        end
        C_tra_1 = C_tra_1(1);
        C_tra_2 = C_tra_2(1);
        Gaps(i).traffic_cost = C_tra_1 + C_tra_2;
    end
end
end