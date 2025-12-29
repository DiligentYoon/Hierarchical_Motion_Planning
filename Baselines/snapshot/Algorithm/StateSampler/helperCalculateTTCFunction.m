function C_t = helperCalculateTTCFunction(ego_gap, target_gap, mioTraj, dt, s_value, d_value, s_dot_value, DurationTime, mapInfo, actorsprofile)

numTrajPoints = length(dt);
LatBoundary = mapInfo.LaneLateralBoundary;
ego_lane = ego_gap.lane_idx;
ego_length = actorsprofile(1).Length;
target_lane = target_gap.lane_idx;
ego_lane_lat = LatBoundary(ego_lane, :);
TTCMax = 5;
TTCMin = 2.2;
C_t = 0;

if ego_lane == target_lane
    ego_mio = find(mioTraj.MIO_lane == ego_lane);
    if ~isempty(ego_mio)
        mio_length = actorsprofile(mioTraj.TargetIDs(ego_mio)).Length;
        for i = 1:numTrajPoints
            ego_s = s_value(i);
            ego_vel = s_dot_value(i);
            relative_vel = ego_vel - mioTraj.MIO_vel(ego_mio);
            if mioTraj.isChange(ego_mio)
                if i < mioTraj.changeidx
                    continue;
                end
            end
            ego_front = ego_s + ego_length/2;
            front_rear = mioTraj.Trajectory(ego_mio, i) - mio_length/2;
            SafetyDistance = front_rear - ego_front;
            if relative_vel > 0 && SafetyDistance > 0
                TTC = SafetyDistance / relative_vel;
                if TTC <= TTCMax
                    C_t = C_t + ((TTCMax - TTC) / (TTCMax - TTCMin))^2;
                end
            end
        end
    else
        C_t = 0;
    end
else
    for i = 1:numTrajPoints
        ego_s = s_value(i);
        ego_d = d_value(i);
        ego_vel = s_dot_value(i);
        if ego_lane_lat(2) <= ego_d && ego_d <= ego_lane_lat(1)
            % 현재 차선
            trajid = find(mioTraj.MIO_lane == ego_lane);
            if ~isempty(trajid)
                mio_length = actorsprofile(mioTraj.TargetIDs(trajid)).Length;
                if mioTraj.isChange(trajid)
                    if i < mioTraj.changeidx
                        continue;
                    end
                end
                relative_vel = ego_vel - mioTraj.MIO_vel(trajid);
                ego_front = ego_s + ego_length/2;
                front_rear = mioTraj.Trajectory(trajid, i) - mio_length/2;
                SafetyDistance = front_rear - ego_front;
                if relative_vel > 0 && SafetyDistance > 0
                    TTC = SafetyDistance / relative_vel;
                    if TTC <= TTCMax
                        C_t = C_t + 0.1*((TTCMax - TTC) / (TTCMax - TTCMin))^2;
                    end
                end
            end
        else
            % 변경 차선
            trajid = find(mioTraj.MIO_lane == target_lane);
            if ~isempty(trajid)
                mio_length = actorsprofile(mioTraj.TargetIDs(trajid)).Length;
                if mioTraj.isChange(trajid)
                    if i < mioTraj.changeidx
                        continue;
                    end
                end
                relative_vel = ego_vel - mioTraj.MIO_vel(trajid);
                ego_front = ego_s + ego_length/2;
                front_rear = mioTraj.Trajectory(trajid, i) - mio_length/2;
                SafetyDistance = front_rear - ego_front;
                if relative_vel > 0 && SafetyDistance > 0
                    TTC = SafetyDistance / relative_vel;
                    if TTC <= TTCMax
                        C_t = C_t + ((TTCMax - TTC) / (TTCMax - TTCMin))^2;
                    end
                end
            end
        end
    end
    C_t = C_t / DurationTime;
end