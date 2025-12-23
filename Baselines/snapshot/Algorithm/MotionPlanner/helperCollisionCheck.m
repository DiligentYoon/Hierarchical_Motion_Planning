function [a_valid, b_valid, result, num_ncp, not_collision_path, mioTraj] = helperCollisionCheck(RMC, target_gap, ego_gap,...
                                                                                        mio_info, ego_info, mio_traj, a, b, params,...
                                                                                            actorsprofile)
%%% mio_traj : Global Frenet 좌표
%%% mio_info : Global Frenet 좌표
%%% ego_info : Global Frenet 좌표

result = true;
not_collision_path = zeros(1, 6);
a_valid = zeros(6, 6);
b_valid = zeros(5, 6);
num_ncp = 6;

numMIOs = mio_traj.NumTrajs;
ego_lane = ego_gap.lane_idx;
target_lane = target_gap.lane_idx;

actors_length = [actorsprofile.Length];
ego_length  = actors_length(1);
vehicle_wide = mean([actorsprofile.Width]);

timestep = params.TimeResolution;
timeHorizon = params.TimeHorizon;
timesample = length(timeHorizon);
numTrajPoints = 31;
velocitysample = params.velocityHorizon;

ego_lane_mio_traj = mio_traj.Trajectories;
change_lane_mio_traj = mio_traj.Trajectories;
MIO_id = 0;

mioTraj.numMIO = 0;
mioTraj.MIO_lane = zeros(10, 1);
mioTraj.MIO_vel = zeros(10, 1);
mioTraj.isChange = zeros(10, 1);
mioTraj.changeidx = 0;
mioTraj.TargetIDs = zeros(10, 1);
mioTraj.Trajectory = zeros(10, numTrajPoints);



if numMIOs == 0
    a_valid = a;
    b_valid = b;
    not_collision_path = 1:1:num_ncp;
else
    n = 1;
    if ego_lane == target_lane
        % 직선 경로..
        % Front 에 존재하는 차량들 가져오기
        ego_lane_mio_idx = find(mio_info(5, 1:numMIOs) == ego_lane);
        ego_lane_mio_filtered_idx = ego_lane_mio_idx(mio_info(1, ego_lane_mio_idx) > ego_info(1));
        if ~isempty(ego_lane_mio_filtered_idx)
            % 가장 가까운 mio 찾고 그 mio의 Trajectory 가져오기
            [~, MIO_idx] = min(mio_info(1, ego_lane_mio_filtered_idx));
            MIO_idx = ego_lane_mio_filtered_idx(MIO_idx);
            MIO_id = mio_info(4, MIO_idx);
            traj_id = find(mio_traj.TargetIDs(1:numMIOs) == MIO_id);
            ego_lane_mio_traj = mio_traj.Trajectories(traj_id);

            mioTraj.Trajectory(1,:) = ego_lane_mio_traj.Trajectory(:, 1)';
            mioTraj.MIO_lane(1) = ego_lane;
            mioTraj.MIO_vel(1) = mio_info(2, MIO_idx);
            mioTraj.TargetIDs(1) = MIO_id;
            mioTraj.numMIO = 1;
            for i = 1:timesample
                % mio의 trajectory는 이미 0.1s 간격으로 정의돼 있음 -> ego 차량만 계산하고 비교..
                tidx = 10 * i+1;
                ego_lane_mio_rear = ego_lane_mio_traj.Trajectory(tidx, 1) - actors_length(MIO_id)/2;
                for j = 1:velocitysample
                    idx = 2*(i-1)+j;
                    ego_front = b(5, idx)*i^4 + b(4, idx)*i^3 + b(3, idx)*i^2 +...
                                b(2, idx)*i + b(1, idx) + ego_length/2;
    
                    if ego_front < ego_lane_mio_rear
                        not_collision_path(n) = n;
                        n = n+1;
                    end
                end
            end
        else
            not_collision_path(1,:) = 1:length(a);
            n = 7;
        end
    
    else
        % 곡선 경로 (차선 변경 O, Target_lane ~= ego_lane)
        % 현재 차선에서의 y값이 범위내에 있을 때의 종방향 위치와, 변경 차선에서의 y값이 범위 내에 있을 때의 종방향 위치.
        % 현재 차선일때는 현재 차선의 앞쪽에 존재하는 차량이 MIO, 변경 차선에서는 변경 차선의 앞 or 뒤에 존재하는 차량 모두 MOI Candidates.
        present_lane_y_left  = ego_gap.d + vehicle_wide/2;
        present_lane_y_right = ego_gap.d - vehicle_wide/2;
        change_lane_y_left  = target_gap.d + vehicle_wide/2;
        change_lane_y_right = target_gap.d - vehicle_wide/2;
    
        ego_lane_mio_idx    = find(mio_info(5, 1:numMIOs) == ego_lane);
        ego_lane_mio        = ego_lane_mio_idx(mio_info(1, ego_lane_mio_idx) > ego_info(1));
        change_lane_mio     = find(mio_info(5, 1:numMIOs) == target_lane);
    
        %%%%%%%%%%%%%%%%%%%%%%%% Initial Setting %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        if (isempty(ego_lane_mio) && isempty(change_lane_mio))
            %%% 현재 차선과 변경 차선 모두 mio가 없는 경우
            a_valid = a;
            b_valid = b;
            not_collision_path = 1:1:num_ncp;
            return;
        elseif ~isempty(ego_lane_mio) && isempty(change_lane_mio)
            %%% 현재 차선에만 mio가 존재하는 경우
            [~, MIO_idx] = min(mio_info(1, ego_lane_mio));
            MIO_idx = ego_lane_mio(MIO_idx);
            MIO_id = mio_info(4, MIO_idx);
            traj_id = find(mio_traj.TargetIDs == MIO_id);
            ego_lane_mio_traj = mio_traj.Trajectories(traj_id);

            mioTraj.Trajectory(1, :) = ego_lane_mio_traj.Trajectory(:, 1)';
            mioTraj.MIO_lane(1) = ego_lane;
            mioTraj.MIO_vel(1) = mio_info(2, MIO_idx);
            mioTraj.TargetIDs(1) = MIO_id;
            mioTraj.numMIO = 1;
        elseif isempty(ego_lane_mio) && ~isempty(change_lane_mio)
            %%% 변경 차선에만 mio가 존재하는 경우
            %%% Ego 차량 기준 앞, 뒤로 가장 가까운 차량 Select
            num_change_mio = length(change_lane_mio);
            if num_change_mio > 1
                change_mio_front = find(mio_info(1, change_lane_mio) > ego_info(1));
                change_mio_rear = find(mio_info(1, change_lane_mio) < ego_info(1));

                [~, change_mio_front_idx] = min(mio_info(1, change_mio_front));
                [~, change_mio_rear_idx] = max(mio_info(1, change_mio_rear));

                change_front_MIO_id = mio_info(4, change_mio_front(change_mio_front_idx));
                change_rear_MIO_id = mio_info(4, change_mio_rear(change_mio_rear_idx));

                change_MIO_id = [change_front_MIO_id, change_rear_MIO_id];
            else
                change_MIO_id = mio_info(4, change_lane_mio);
            end
            num_change_mio = length(change_MIO_id);
                
            traj_id = ismember(mio_traj.TargetIDs, change_MIO_id);
            traj_id = find(traj_id == 1);
            change_lane_mio_traj = mio_traj.Trajectories(traj_id);
            change_lane_mio_vel = mio_info(2, change_lane_mio);

            for i = 1:num_change_mio
                mioTraj.Trajectory(i, :) = change_lane_mio_traj(i).Trajectory(:, 1);
                mioTraj.MIO_lane(i) = target_lane;
                mioTraj.MIO_vel(i) = change_lane_mio_vel(i);
                mioTraj.TargetIDs(i) = change_MIO_id(i);
            end
            mioTraj.numMIO = num_change_mio;
        else
            %%% 두개 차선에 모두 mio가 존재하는 경우
            %%% 변경 차선에는 MIO가 2대일 가능성 존재
            num_change_mio = length(change_lane_mio);
            [~, MIO_idx] = min(mio_info(1, ego_lane_mio));
            MIO_idx = ego_lane_mio(MIO_idx);
            MIO_id = mio_info(4, MIO_idx);
            traj_id = find(mio_traj.TargetIDs == MIO_id);
            ego_lane_mio_traj = mio_traj.Trajectories(traj_id);
        
            if num_change_mio > 1
                change_mio_front = find(mio_info(1, change_lane_mio) > ego_info(1));
                change_mio_rear = find(mio_info(1, change_lane_mio) < ego_info(1));

                [~, change_mio_front_idx] = min(mio_info(1, change_mio_front));
                [~, change_mio_rear_idx] = max(mio_info(1, change_mio_rear));

                change_front_MIO_id = mio_info(4, change_mio_front(change_mio_front_idx));
                change_rear_MIO_id = mio_info(4, change_mio_rear(change_mio_rear_idx));

                change_MIO_id = [change_front_MIO_id, change_rear_MIO_id];
            else
                change_MIO_id = mio_info(4, change_lane_mio);
            end
            num_change_mio = length(change_MIO_id);

            traj_id = ismember(mio_traj.TargetIDs, change_MIO_id);
            traj_id = find(traj_id == 1);
            change_lane_mio_traj = mio_traj.Trajectories(traj_id);
            change_lane_mio_vel = mio_info(2, change_lane_mio);

            mioTraj.Trajectory(1, :) = ego_lane_mio_traj.Trajectory(:, 1)';
            mioTraj.MIO_lane(1) = ego_lane;
            mioTraj.MIO_vel(1) = mio_info(2, MIO_idx);
            mioTraj.TargetIDs(1) = MIO_id;
            for i = 1:num_change_mio
                mioTraj.Trajectory(i+1,:) = change_lane_mio_traj(i).Trajectory(:, 1)';
                mioTraj.MIO_lane(i+1) = target_lane;
                mioTraj.MIO_vel(i+1) = change_lane_mio_vel(i);
                mioTraj.TargetIDs(i+1) = change_MIO_id(i);
            end
            mioTraj.numMIO = 1 + num_change_mio;
        end
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        for i = 1:timesample
            tidx = (10 * i)+1;
            for j = 1:velocitysample
                collision = false;
                for k = 1:tidx
                    dt = timestep*(k-1);
                    % idx = 3*(i-1)+j;
                    idx = 2*(i-1)+j;
    
                    d_t = a(6, idx)*dt^5 + a(5, idx)*dt^4 + a(4, idx)*dt^3 +...
                                a(3, idx)*dt^2 + a(2, idx)*dt + a(1, idx);
                    s_t = b(5, idx)*dt^4 + b(4, idx)*dt^3 + b(3, idx)*dt^2 +...
                                b(2, idx)*dt + b(1, idx);
    
                    if ~isempty(ego_lane_mio)
                        if d_t <= present_lane_y_left && d_t >= present_lane_y_right
                            ego_front = s_t + ego_length/2;
                            ego_lane_mio_rear = ego_lane_mio_traj.Trajectory(k, 1) - actors_length(MIO_id)/2;
                            if ego_front >= ego_lane_mio_rear
                                collision = true;
                                break;
                            end
                        end
                    end
    
                    if ~isempty(change_lane_mio)
                        if d_t <= change_lane_y_left && d_t >= change_lane_y_right
                            ego_rear = s_t - ego_length/2;
                            ego_front = s_t + ego_length/2;
                            for m = 1:length(change_lane_mio)
                                change_lane_mio_rear = change_lane_mio_traj(m).Trajectory(k, 1) -...
                                                       actors_length(mio_info(4, change_lane_mio(m)))/2;
                                change_lane_mio_front = change_lane_mio_traj(m).Trajectory(k, 1) + ...
                                                        actors_length(mio_info(4, change_lane_mio(m)))/2;
        
                                if change_lane_mio_front < ego_front
                                    if ego_rear <= change_lane_mio_front
                                        collision = true;
                                        break;
                                    end
                                else
                                    if ego_front >= change_lane_mio_rear
                                        collision = true;
                                        break;
                                    end
                                end
                            end
                            if collision
                                break;
                            end
                         end
                     end
                 end
                 if ~collision
                     not_collision_path(n) = n;
                     n = n+1;
                 end
            end
        end
    end
    
    if n ~= 1
        num_ncp = n-1;
        a_valid(:, 1:num_ncp) = a(:, not_collision_path(1:num_ncp));
        b_valid(:, 1:num_ncp) = b(:, not_collision_path(1:num_ncp));
    else
        a_valid(:, 1) = a(:, 6);
        b_valid(:, 1) = b(:, 6);
        result = false;
    end
end                                                