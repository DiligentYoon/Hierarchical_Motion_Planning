%% ExcelFile Definitions
params = struct("MaxLonVel", maxLonVelocity, ...
                "MaxLatVel", maxLatVelocity, ...
                "MinLonVel", minLonVelocity, ...
                "MaxLonAccel", maxLonAccel, ...
                "MaxLatAccel", maxLatAccel, ...
                "MaxFront", MaxFront, ...
                "MaxTTC", MaxTTC, ...
                "MinTTC", MinTTC);

excelFiles = ["Results/result_w_case1.xlsx", ...
              "Results/result_wo_case1.xlsx", ...
              "Results/result_w_case3.xlsx", ...
              "Results/result_wo_case3.xlsx"];


%% Data Extraction
[s1_w, ts1_w] = load_sim_results(excelFiles(1));
[s1_wo, ts1_wo] = load_sim_results(excelFiles(2)); 

[s3_w, ts3_w] = load_sim_results(excelFiles(3));
[s3_wo, ts3_wo] = load_sim_results(excelFiles(4)); 


%% Plot
close all;
% Scenario 1: (2 x 2) subplot
figure; set(gcf, 'color', 'w', 'Position', [100 100 1800 600]);

% ---- helper: trigger event time (safety trigger only, skip 3s periodic) ----
timeTrigPeriod = 3.0;   % time-expiration trigger 주기
timeTrigTol    = 0.15;  % "3초" 판별 허용 오차 

% ========== (1,1) TTC with manager ==========
subplot(2,2,1);
plot_ttc_with_triggers(ts1_w, params, timeTrigPeriod, timeTrigTol);
title('Ablation 2 - TTC (with manager)');

% ========== (1,2) Safety Distance with manager ==========
subplot(2,2,2);
plot_safety_with_triggers(ts1_w, params, timeTrigPeriod, timeTrigTol);
title('Ablation 2 - Safety distance (with manager)');

% ========== (2,1) TTC without manager ==========
subplot(2,2,3);
plot_ttc_with_triggers(ts1_wo, params, timeTrigPeriod, timeTrigTol);
title('Ablation 2 - TTC (without manager)');

% ========== (2,2) Safety Distance without manager ==========
subplot(2,2,4);
plot_safety_with_triggers(ts1_wo, params, timeTrigPeriod, timeTrigTol);
title('Ablation 2 - Safety distance (without manager)');


% Scenario 3: (2 x 2) subplot
figure; set(gcf, 'color', 'w', 'Position', [100 100 1800 600]);

% ========== (1,1) TTC with manager ==========
subplot(2,2,1);
plot_ttc_with_triggers(ts3_w, params, timeTrigPeriod, timeTrigTol);
title('Ablation 1 - TTC (with manager)');

% ========== (1,2) Safety Distance with manager ==========
subplot(2,2,2);
plot_safety_with_triggers(ts3_w, params, timeTrigPeriod, timeTrigTol);
title('Ablation 1 - Safety distance (with manager)');

% ========== (2,1) TTC without manager ==========
subplot(2,2,3);
plot_ttc_with_triggers(ts3_wo, params, timeTrigPeriod, timeTrigTol);
title('Ablation 1 - TTC (without manager)');

% ========== (2,2) Safety Distance without manager ==========
subplot(2,2,4);
plot_safety_with_triggers(ts3_wo, params, timeTrigPeriod, timeTrigTol);
title('Ablation 1 - Safety distance (without manager)');



%% ------------------------------------------------------------------------

function plot_ttc_with_triggers(ts, params, timeTrigPeriod, timeTrigTol)
    time = ts.time;
    ttc  = ts.ttc;

    % 상한 클리핑
    ttc_plot = min(ttc, params.MaxTTC);

    % 포화 구간 마스크 (TTC == MaxTTC 근처)
    sat_mask = (ttc >= params.MaxTTC - 1e-6);

    % 1) 전체 구간을 점선으로 먼저 그림
    plot(time, ttc_plot, 'b--', 'LineWidth', 0.5); hold on;

    % 2) 포화가 아닌 구간만 진한 빨간색으로 덧그리기
    ttc_active = ttc_plot;
    ttc_active(sat_mask) = NaN;   % active 구간만 남김
    plot(time, ttc_active, 'r', 'LineWidth', 1.5);

    % 기준선들
    yline(params.MinTTC, 'm--', 'MinTTC', 'LabelHorizontalAlignment', 'left');
    yline(params.MaxTTC, 'g--', 'MaxTTC', 'LabelHorizontalAlignment', 'left');
    yline(0, 'r--', 'Safety Margin Violation', 'LabelHorizontalAlignment','left');

    % Trigger 마킹 (safety trigger만)
    trig_times = get_safety_trigger_times(time, ts.trigger, timeTrigPeriod, timeTrigTol);
    if ~isempty(trig_times)
        ttc_at_trig = interp1(time, ttc_plot, trig_times, 'linear', 'extrap');
        scatter(trig_times, ttc_at_trig, 50, 'r', 'filled', 'v');
    end

    xlabel('Time (s)');
    ylabel('TTC (s)');
    grid on;
    xlim([time(1), time(end)]);
    ylim([-1, params.MaxTTC + 1]);

    % 필요하면 legend도 추가 가능
    % legend({'TTC (saturated)','TTC (active region)','Trigger'}, 'Location','best');
end


function plot_safety_with_triggers(ts, params, timeTrigPeriod, timeTrigTol)
    time = ts.time;
    sd   = ts.safety_distance;

    % 포화 기준: 150 (또는 params.MaxFront 사용)
    sat_mask = (sd >= 150 - 1e-6);

    % 1) 전체 구간을 점선으로 그림
    plot(time, sd, 'b--', 'LineWidth', 0.5); hold on;

    % 2) 포화가 아닌 구간만 진한 빨간색으로 덧그리기
    sd_active = sd;
    sd_active(sat_mask) = NaN;
    plot(time, sd_active, 'r', 'LineWidth', 1.5);
    
    yline(150, 'g--', 'Max Front Margin', 'LabelHorizontalAlignment', 'left')
    yline(params.MaxFront, 'r--', 'Safety Margin Violation', 'LabelHorizontalAlignment', 'left');

    % Trigger 마킹
    trig_times = get_safety_trigger_times(time, ts.trigger, timeTrigPeriod, timeTrigTol);
    if ~isempty(trig_times)
        sd_at_trig = interp1(time, sd, trig_times, 'linear', 'extrap');
        scatter(trig_times, sd_at_trig, 50, 'r', 'filled', 'v');
    end

    xlabel('Time (s)');
    ylabel('Safety distance (m)');
    grid on;
    xlim([time(1), time(end)]);
    ylim([0, 170]);
end


function trig_times = get_safety_trigger_times(time, trigger, period, tol)
    % trigger: 0/1 신호 (any trigger)
    % period: time-expiration trigger 주기 (예: 3초)
    % tol   : 주기 판별 허용오차
    %
    % 역할:
    %  1) 0 -> 1 rising edge 위치 찾기
    %  2) 그 중 "거의 period(±tol) 간격으로 반복되는 edge"들을
    %     time-expiration trigger로 보고 제거
    %  3) 남은 edge time 들을 safety trigger로 간주하여 반환

    if isempty(trigger)
        trig_times = [];
        return;
    end

    % 1) rising edge 추출
    trig = trigger(:);  % column vector 보장
    idx_all = find(trig(1:end-1) <= 0.5 & trig(2:end) > 0.5) + 1;
    if isempty(idx_all)
        trig_times = [];
        return;
    end
    t_all = time(idx_all);

    % 2) time-expiration (periodic) trigger 판별
    %    - 연속 edge 간 시간차가 period 근처면 periodic으로 판단
    dt = diff(t_all);
    is_periodic_gap = abs(dt - period) < tol;  % true면 periodic pattern 후보

    keep = true(size(t_all));  % 최종적으로 남길 edge 마스크

    % dt(k)는 t_all(k) -> t_all(k+1) 간격
    % 이 간격이 periodic이면 두 edge 모두 time-trigger로 보고 제거
    for k = 1:numel(is_periodic_gap)
        if is_periodic_gap(k)
            keep(k)   = false;
            keep(k+1) = false;
        end
    end

    trig_times = t_all(keep);
end


function [summary, ts] = load_sim_results(excelFile)
    % Summary 시트
    summaryTbl = readtable(excelFile, 'Sheet', 'Summary');
    summary.mean_ttc        = summaryTbl.MeanTTC(1);
    summary.num_ttc         = summaryTbl.NumTTC(1);
    summary.travel_lon_dist = summaryTbl.TravelLonDist(1);
    summary.travel_lat_dist = summaryTbl.TravelLatDist(1);

    % TimeSeries 시트
    tsTbl = readtable(excelFile, 'Sheet', 'TimeSeries');
    ts.time             = tsTbl.Time;
    ts.lon_vel          = tsTbl.LonVel;
    ts.lat_vel          = tsTbl.LatVel;
    ts.lon_accel        = tsTbl.LonAccel;
    ts.lat_accel        = tsTbl.LatAccel;
    ts.ttc              = tsTbl.TTC;
    ts.safety_distance  = tsTbl.SafetyDistance;
    ts.trigger          = tsTbl.Trigger;
end