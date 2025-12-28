%%% This Script is used to plot Decision Making and Trajectory Planning

%%% From Simulink, Simulation Results are below:
%%%     1. EgoFrenet : Frenet State of Ego Vehicle [s, ds, dds, l, dl, ddl]
%%%     2. EgoInfo: Cartesian State of Ego Vehicle in Ego Coordinate [x y z vx, vy, vz, ax, ay, az, yaw, yaw_rate]
%%%     3. TTC : Time-to-Collision between Ego and MIO Vehicles
%%%     4. Trigger : Trigger signal related to Event-Triggered Manager
%%%     5. Trajectory : Optimal Trajectory

dt = 0.1;
refPath = mapInfo.GlobalPlanPoints(1:mapInfo.NumGlobalPlanPoints,:);
RefPathFrenet = referencePathFrenet(refPath); 
params = struct("MaxLonVel", maxLonVelocity, ...
                "MaxLatVel", maxLatVelocity, ...
                "MinLonVel", minLonVelocity, ...
                "MaxLonAccel", maxLonAccel, ...
                "MaxLatAccel", maxLatAccel, ...
                "MaxFront", MaxFront, ...
                "MaxTTC", MaxTTC, ...
                "MinTTC", MinTTC);

% Data Extraction
time = tout;
ego_frenet = squeeze(EgoFrenet.signals.values);
ego_info = squeeze(EgoInfo.signals.values);
inverse_ttc = inverse_TTC.signals.values;
trigger = Trigger.signals.values;
trajectory = Trajectory.signals.values;
safety_distance = Safety_Distance.signals.values;


%% Plot
close all;

% 1. Plot Ego Vehicle Velocity (Lon and Lat Directions)
figure;
set(gcf, 'color', 'w')

approx_lat_vel = zeros(length(time), 1);
approx_lat_vel(1) = (ego_frenet(4, 2) - ego_frenet(4, 1)) / dt;
approx_lat_vel(end) = (ego_frenet(4, end) - ego_frenet(4, end-1)) / dt;
approx_lat_vel(2:end-1) = (ego_frenet(4, 3:end) - ego_frenet(4, 1:end-2)) ./ (2*dt);


subplot(2,1,1);
plot(time, ego_frenet(2, :), 'LineWidth', 1.5);
hold on;
yline(params.MaxLonVel, 'r--', 'MaxLonVel', 'LabelHorizontalAlignment', 'left');
yline(params.MinLonVel, 'r--', 'MinLonVel', 'LabelHorizontalAlignment', 'left');
xlabel('Time (s)'); 
ylabel('Velocity (m/s)');
title('Ego Vehicle Longitudinal Velocity');
grid on;

subplot(2,1,2);
plot(time, approx_lat_vel, 'LineWidth', 1.5);
hold on;
yline(params.MaxLatVel, 'r--', 'MaxLatVel', 'LabelHorizontalAlignment', 'left');
yline(-params.MaxLatVel, 'r--', 'MinLatVel', 'LabelHorizontalAlignment', 'left');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Ego Vehicle Lateral Velocity');
grid on;

% 2. Plot Ego Vehicle Acceleration (Lon and Lat Directions)
figure;
set(gcf, 'color', 'w')

approx_lon_accel = zeros(length(time), 1);
approx_lon_accel(1) = (ego_frenet(2, 2) - ego_frenet(2, 1)) / dt;
approx_lon_accel(end) = (ego_frenet(2, end) - ego_frenet(2, end-1)) / dt;
approx_lon_accel(2:end-1) = (ego_frenet(2, 3:end) - ego_frenet(2, 1:end-2)) ./ (2*dt);

approx_lat_accel = zeros(length(time), 1);
approx_lat_accel(1) = (approx_lat_vel(2) - approx_lat_vel(1)) / dt;
approx_lat_accel(end) = (approx_lat_vel(end) - approx_lat_vel(end-1)) / dt;
approx_lat_accel(2:end-1) =  (approx_lat_vel(3:end) - approx_lat_vel(1:end-2)) ./ (2*dt);

subplot(2,1,1);
plot(time, approx_lon_accel, 'LineWidth', 1.5);
hold on;
yline(params.MaxLonAccel, 'r--', 'MaxLonAccel', 'LabelHorizontalAlignment', 'left');
yline(-params.MaxLonAccel, 'r--', 'MinLonAccel', 'LabelHorizontalAlignment', 'left');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
ylim([-params.MaxLonAccel-1, params.MaxLonAccel+1])
title('Ego Vehicle Longitudinal Acceleration');
grid on;

subplot(2,1,2);
plot(time, approx_lat_accel, 'LineWidth', 1.5);
hold on;
yline(params.MaxLatAccel, 'r--', 'MaxLatAccel', 'LabelHorizontalAlignment', 'left');
yline(-params.MaxLatAccel, 'r--', 'MinLatAccel', 'LabelHorizontalAlignment', 'left');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
ylim([-params.MaxLatAccel-1, params.MaxLatAccel+1])
title('Ego Vehicle Lateral Acceleration');
grid on;


% 3. Plot TTC and Trigger Signal
figure;
set(gcf, 'color', 'w')

hold on;
plot(time, inverse_ttc, 'LineWidth', 1.5);
% plot(time, trigger,  'r--', 'LineWidth', 0.5);
yline(1/params.MinTTC, 'r--', 'MinTTC', 'LabelHorizontalAlignment', 'left');
% yline(1/params.MaxTTC, 'g--', 'MaxTTC', 'LabelHorizontalAlignment', 'left');
ylim([0, 1/params.MinTTC + 0.5])
xlabel('Time (s)');
ylabel('Inverse Time-to-Collision (s)');
title('Inverse Time-to-Collision (TTC)');
grid on;


%% 1. Safety Analysis

ttc_vio_id = (inverse_ttc >= 1/params.MinTTC);
ttc_vio_value = inverse_ttc(ttc_vio_id);

if isempty(ttc_vio_value)
    mean_ittc = 0;
    num_ittc = length(ttc_vio_value);
else
    mean_ittc = mean(ttc_vio_value);
    num_ittc = length(ttc_vio_value);
end
fprintf("TTC violation analysis: \n")
fprintf("mean of vio TTC  : %.2f \n", mean_ittc);
fprintf("Num vio : %d \n", num_ittc);

%% 2. Efficiency Analysis

travel_lon_dist = hypot(ego_info(1, end)-ego_info(1, 1), ego_info(2, end)-ego_info(2, 1));
travel_lat_dist = sum(abs(diff(ego_frenet(4, :))));
fprintf("Efficiency analysis: \n")
fprintf("travel longitudinal distance : %.2f\n", travel_lon_dist);
fprintf("total lateral distance : %.2f\n", travel_lat_dist);


%% Excel
excelFile = 'Results/result_wo_case2.xlsx';

% Summary metrics 

summaryTbl = table(mean_ittc, num_ittc, travel_lon_dist, travel_lat_dist, ...
    'VariableNames', {'MeanITTC', 'NumITTC', 'TravelLonDist', 'TravelLatDist'});

writetable(summaryTbl, excelFile, 'Sheet', 'Summary', 'WriteMode', 'overwritesheet');

timeSeriesTbl = table( ...
    time(:), ...
    ego_frenet(2, :).', ...        % Longitudinal velocity
    approx_lat_vel(:), ...         % Lateral velocity
    approx_lon_accel(:), ...       % Longitudinal acceleration
    approx_lat_accel(:), ...       % Lateral acceleration
    inverse_ttc(:), ...            % Inverse TTC
    safety_distance(:), ...        % Safety Distance
    trigger(:), ...                % Trigger
    'VariableNames', { ...
        'Time', ...
        'LonVel', ...
        'LatVel', ...
        'LonAccel', ...
        'LatAccel', ...
        'ITTC', ...
        'SafetyDistance', ...
        'Trigger'});

% "TimeSeries" 시트에 저장
writetable(timeSeriesTbl, excelFile, 'Sheet', 'TimeSeries', 'WriteMode', 'overwritesheet');