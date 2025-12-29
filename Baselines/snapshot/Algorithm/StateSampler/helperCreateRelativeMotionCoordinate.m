function RMC = helperCreateRelativeMotionCoordinate(mioRelativeFrenetStates, ...
                                                    mioGlobalFrenetStates, ...
                                                    FutureRelativeTrajectory, ...
                                                    MapInfo, ...
                                                    initStruct, ...
                                                    Actors, ...
                                                    EgoActor, EgoLane)
%%% mioRelativeFrenetStates %%%
%%% 1. NumMIOs      : MIO 개수
%%% 2. TargetIds    : MIO 차량들의 고유 번호
%%% 3. MIOStates    : MIO 차량들의 Frenet States w.r.t Relative (s ds dds l dl ddl)

%%% mioGlobalFrenetStates %%%
%%% NumMIOs     : MIO 개수
%%% TargetIds   : MIO 차량들의 고유 번호
%%% MIOStates   : MIO 차량들의 Frenet States w.r.t Global (s ds dds l dl ddl)

%%% FutureRelativeTrajectory %%%
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

MIOIds = mioRelativeFrenetStates.TargetIds;
NumMIOs = mioRelativeFrenetStates.NumMIOs;
NumLanes = MapInfo.NumLanes;
LaneNumber = zeros(1,NumMIOs);
Relativelane = zeros(1, NumMIOs);
laneBoundaries = repmat(struct('MinS', zeros(5, 1), 'MaxS', zeros(5, 1), 'VehNum', 0),NumLanes, 1);
RMC = initStruct;

% 1. 각 차량들이 어느 차선에 있는지 check
% 2. MIOIds, LaneNumber -> 각 차량이 몇 차선에 있는가? 나옴.. 이를 토대로 RMC
% 3. Boundary 설정

for i = 1:NumMIOs
    MIO_lat = mioGlobalFrenetStates.MIOStates(i, 4);
    LaneNumber(i) = helperDetectLaneNumber(MapInfo, MIO_lat);
    Relativelane(i) = round(mioRelativeFrenetStates.MIOStates(i, 4) / MapInfo.LaneWidth(1));
end
% MIO 기준으로 Relative하게 Ego 차선을 구하다 보니, MIO가 현재 차선에 존재하지 않으면 에러가 남
% 일관성 있게 Ego 차량의 차선을 구할 수 있는 방법이 무엇일까?
% + 코드 최적화
% 차량의 yaw 각도와 길이 그리고 중심점 위치를 받아오면 정확한 위치를 구할 수 있음
egolane = EgoLane;

% fprintf("Ego Lane : %i\n", int32(egolane));

n = 1;
for lane = 1:NumLanes
    % 해당 차선에 있는 MIO의 인덱스
    MIOIndicesInLane = find(LaneNumber == lane);
    if isempty(MIOIndicesInLane)
        continue;
    end
    
    % 예측된 경로에서 종방향 위치를 수집
    for i = 1:length(MIOIndicesInLane)
        idx = MIOIndicesInLane(i);
        MIO_ID = MIOIds(idx);
        trajIdx = find(FutureRelativeTrajectory.TargetIDs == MIO_ID);
        traj = FutureRelativeTrajectory.Trajectories(trajIdx).Trajectory;
        ds = traj(1, 2);
        sStart = traj(1, 1);
        sEnd = traj(end, 1);
        mins = min(sStart, sEnd);
        maxs = max(sStart, sEnd);
        
        % 해당 MIO 차량의 길이 가져오기
        actorID = MIOIds(idx);
        actor = Actors([Actors.ActorID] == actorID);
        vehicleLength = actor.Length;
        
        % 종방향 경계 계산
        minS = max(-50, mins - vehicleLength/2);
        maxS = min(50, maxs + vehicleLength/2);

        if lane == egolane
            if ds > 0
                if sStart < 0 && sEnd > -(EgoActor.Length + vehicleLength)/2
                    maxS = -EgoActor.Length/2;
                end
            else
                if sStart > 0 && sEnd < (EgoActor.Length + vehicleLength)/2
                    minS = EgoActor.Length/2;
                end
            end
        end
        
        laneBoundaries(lane).MinS(i) = minS;
        laneBoundaries(lane).MaxS(i) = maxS;
        RMC.veh_Ids(n) = MIO_ID;
        n = n+1;
    end
    laneBoundaries(lane).VehNum = length(MIOIndicesInLane);
end

% 3. 각 차선별로 Gap을 계산하고 RMC 구조체에 추가
gapIdx = 1;
for lane = 1:NumLanes

    if abs(egolane - lane) > 2
        % 2차선 이상 -> Unvalid Lane !
        continue;
    end

    if (laneBoundaries(lane).VehNum == 0)
        % 해당 차선에 MIO가 없을 경우 전체 범위가 Gap
        RMC.Gaps(gapIdx).idx = gapIdx;
        RMC.Gaps(gapIdx).lane_idx = lane;
        RMC.Gaps(gapIdx).s = 0;
        RMC.Gaps(gapIdx).d = MapInfo.LaneCenters(lane);
        RMC.Gaps(gapIdx).l = 100;
        RMC.Gaps(gapIdx).IsValid = true;
        RMC.veh_number(lane) = 0;
        gapIdx = gapIdx + 1;
        continue;
    end


    
    % Boundary 정렬
    [sortedMinS, sortIdx] = sort(laneBoundaries(lane).MinS);
    sortedMaxS = laneBoundaries(lane).MaxS(sortIdx);
    sortedMinS = sortedMinS(sortedMinS ~= 0);
    sortedMaxS = sortedMaxS(sortedMaxS ~= 0);
    
    lane_gaps = [];
    % 첫 번째 Gap (첫 Boundary 전)
    if sortedMinS(1) > -50
        lane_gaps = [-50, sortedMinS(1)];
    end
    
    % 중간 Gap
    for i = 1:length(sortedMaxS) - 1
        if sortedMaxS(i) < sortedMinS(i+1)
            lane_gaps = [lane_gaps; sortedMaxS(i), sortedMinS(i+1)];
        end
    end
    
    % 마지막 Gap (마지막 Boundary 후)
    if sortedMaxS(end) < 50
        lane_gaps = [lane_gaps; sortedMaxS(end), 50];
    end
    
    % 각 Gap을 RMC 구조체에 추가
    for i = 1:size(lane_gaps, 1)
        gapCenterS = (lane_gaps(i, 1) + lane_gaps(i, 2)) / 2;
        gapLength = lane_gaps(i, 2) - lane_gaps(i, 1);
        RMC.Gaps(gapIdx).idx = gapIdx;
        RMC.Gaps(gapIdx).lane_idx = lane;
        RMC.Gaps(gapIdx).s = gapCenterS;
        RMC.Gaps(gapIdx).d = MapInfo.LaneCenters(lane);
        RMC.Gaps(gapIdx).l = gapLength;
        RMC.Gaps(gapIdx).IsValid = false;
        gapIdx = gapIdx + 1;
    end
    RMC.veh_number(lane) = laneBoundaries(lane).VehNum;
end
RMC.EgoLane = egolane;
RMC.NumGaps = gapIdx - 1;

end