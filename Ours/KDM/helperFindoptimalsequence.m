function Optimal_Planner = helperFindoptimalsequence(InitStruct_Planner, ...
                                                     node_costs, ...
                                                     LK_set, ...
                                                     LC_set, ...
                                                     Planner_params)
    % 제약
    %  - 연속 Spatio 금지
    %  - Spatio 1회만 허용
    %  - Spatio일 때만 동역학 타당성 검사(dynamical_feasible)

    Optimal_Planner  = InitStruct_Planner;
    num_total_nodes  = Optimal_Planner.num_total_nodes;
    Gaps             = Optimal_Planner.Gaps(1:num_total_nodes);
    W                = Optimal_Planner.adjacency_matrix(1:num_total_nodes, 1:num_total_nodes);
    start_node       = Optimal_Planner.start_node;
    end_nodes_all    = Optimal_Planner.end_nodes(:).';   % row vector
    end_nodes        = end_nodes_all(end_nodes_all ~= 0);
    N                = num_total_nodes;
    maxDeg = 10;
    FALSE = 1;  
    TRUE  = 2;

    % --- 전처리: 이웃 목록 / Spatio 인접표 ---
    [neighbors_mat, neighbors_len] = precomputeNeighbors(W, maxDeg); 
    S = buildSpatioAdjacency(LC_set, N);      % logical(N,N), S(u,v)=true면 spatio

    % --- 자료구조 사전할당 (노드×상태 2×2) ---
    % dist(v,e,l), prev_v/prev_e/prev_l(v,e,l)
    dist   =   inf(N, 2, 2); % Node에 (e, l)상태로 도달했을 때 최소 비용
    prev_v = zeros(N, 2, 2); % node
    prev_e = zeros(N, 2, 2); % everspatio (한 번이라도 차선 변경이 있었는지 ?)
    prev_l = zeros(N, 2, 2); % lastspatio (직전 노드가 차선 변경을 수행했는지 ?)
    inQ    = false(N, 2, 2); % (v, e, l) 상태가 Q 집합 내부에 있는지 ?

    % 시작 상태: (start_node, e=0, l=0)
    dist(start_node, FALSE, FALSE) = 0;
    % dist(start_node, FALSE, FALSE) = node_costs(start_node);
    inQ(start_node,  FALSE, FALSE) = true;

    % --- 제약 포함 다익스트라 메인 루프 ---
    while any(inQ(:))
        % 후보 중 최소 dist 상태 추출
        [u, eu, lu, bestval] = extractMinLabel(inQ, dist);
        if isinf(bestval)
            break;
        end
        inQ(u, eu, lu) = false;

        len_u = min(neighbors_len(u), maxDeg);
        if len_u == 0
            continue;
        end

        lastIsSp_u = (lu == TRUE); % 현재 u 노드는 직전 노드가 차선 변경 노드인가 ?
        everSp_u   = (eu == TRUE); % 현재 u 노드는 한 번이라도 차선 변경이 있었는가 ?

        % 모든 이웃 v 확장
        for k = 1:len_u
            v = neighbors_mat(u, k);
            edgeIsSp = S(u, v); % 이번 연결이 차선 변경 관계인가 ?
            % (A) 연속 Spatio 금지
            if lastIsSp_u && edgeIsSp
                continue;
            end
            % (B) Spatio 1회 제한 + 동역학 타당성(Spatio일 때만)
            if edgeIsSp
                if everSp_u
                    continue;
                end
                if ~dynamical_feasible(Gaps, start_node, v, Planner_params)
                    continue;
                end
            end
            % (C) 다음 상태 계산
            lv = FALSE; if edgeIsSp, lv = TRUE; end
            ev = FALSE; if (everSp_u || edgeIsSp), ev = TRUE; end
            % (D) 비용 갱신 (동일 time index에서 비용 중복 누적 방지)
            if (Gaps(u).time_idx == Gaps(v).time_idx)
                % 동일 time index에서는 차선 변경에 대한 spatio cost만 누적
                node_cost = 0;
            else
                node_cost = node_costs(v);
            end
            alt = bestval + W(u, v) + node_cost;
            if alt < dist(v, ev, lv)
                dist(v, ev, lv)   = alt;
                prev_v(v, ev, lv) = u;
                prev_e(v, ev, lv) = eu;
                prev_l(v, ev, lv) = lu;
                inQ(v, ev, lv)    = true;
            end
        end
    end

    % --- 종점 선택: (v,*,*) 중 최소 비용 ---
    best_end_node = 0;
    min_cost = inf;
    best_e = 0;
    best_l = 0;

    for i = 1:numel(end_nodes)
        v = end_nodes(i);
        for eix = 1:2
            for lix = 1:2
                dval = dist(v, eix, lix);
                if dval < min_cost
                    min_cost = dval;
                    best_end_node = v;
                    best_e = eix;
                    best_l = lix;
                end
            end
        end
    end

    % --- 경로 역추적(라벨 포함) + 정방향 뒤집기 ---
    optimal_path = zeros(1, N);
    plen = 0;

    if best_end_node ~= 0 && isfinite(min_cost)
        cur_v = best_end_node;
        cur_e = best_e;
        cur_l = best_l;

        while cur_v ~= 0
            plen = plen + 1;
            optimal_path(plen) = cur_v;

            pv = prev_v(cur_v, cur_e, cur_l);
            if pv == 0
                break;
            end
            pe = prev_e(cur_v, cur_e, cur_l);
            pl = prev_l(cur_v, cur_e, cur_l);

            cur_v = pv; cur_e = pe; cur_l = pl;
        end

        optimal_path(1:plen) = reverse_slice(optimal_path(1:double(plen)));
    else
        plen = 0;
    end

    % --- 동일 시각(time_idx) 노드 제거(LC 엣지 병합) ---
    if plen > 1
        keepMask = true(1, plen);
        for k = 1:(plen-1)
            g1 = optimal_path(k);
            g2 = optimal_path(k+1);
            if Gaps(g1).time_idx == Gaps(g2).time_idx
                keepMask(k) = false;    % 앞 노드 제거 (spatio 결과 유지)
            end
        end
        tmp  = optimal_path(1:plen);
        tmp  = tmp(keepMask);
        plen = numel(tmp);
        optimal_path(1:plen) = tmp;
    end

    % --- 결과 저장 ---
    num_opt = double(plen);
    if num_opt > 0
        Optimal_Planner.optimal_gap_sequence(1:num_opt) = optimal_path(1:num_opt);
    else
        Optimal_Planner.optimal_gap_sequence(:) = 0;
    end
end

% ======= 보조 함수들 =======

function [neighbors_mat, neighbors_len] = precomputeNeighbors(W, maxDeg)
% 이웃 목록 전처리: isfinite(W(u,:))를 한 번만 계산
    N = size(W,1);
    neighbors_mat = zeros(N, maxDeg);
    neighbors_len = zeros(N, 1);
    for u = 1:N
        cnt = 0;
        % 유효 이웃을 스캔
        for v = 1:N
            if u == v
                continue;
            end
            if isfinite(W(u, v))
                cnt = cnt + 1;
                if cnt <= maxDeg
                    neighbors_mat(u, cnt) = v;
                end
            end
        end
        neighbors_len(u) = cnt;
    end
end

function S = buildSpatioAdjacency(LC_set, N)
    % Spatio 인접표: S(u,v)=true면 spatio edge
    S = false(N, N);
    for i = 1:length(LC_set)
        % 동일 시점의 Spatio Graph를 바탕으로 Marking
        sp = LC_set(i).spatio_topology_global_graph;
        num_nodes = LC_set(i).num_nodes;
        if ~isempty(sp)
            srcs = sp(1, 1:num_nodes);
            dsts = sp(2, 1:num_nodes);
            m = numel(srcs);
            for k = 1:m
                s = srcs(k); d = dsts(k);
                if s >= 1 && s <= N && d >= 1 && d <= N
                    S(s, d) = true;
                end
            end
        end
    end
end

function [u, eu, lu, bestval] = extractMinLabel(inQ, dist)
    % 후보(inQ) 중 dist 최소의 (u,e,l) 선택 (선형 스캔)
    bestval = inf; u = 0; eu = 0; lu = 0;
    [N, ~, ~] = size(dist);
    for eix = 1:2
        for lix = 1:2
            qmask = inQ(:, eix, lix);
            if any(qmask)
                dv = dist(qmask, eix, lix);
                idxs = find(qmask);
                [b, loc] = min(dv);
                if b < bestval
                    bestval = b;
                    u  = idxs(loc);
                    eu = eix;
                    lu = lix;
                end
            end
        end
    end
    if u==0
        bestval = inf;
    end
end

function out = reverse_slice(x)
%#codegen
    n = numel(x);
    out = zeros(size(x));
    for i = 1:n
        out(i) = x(n - i + 1);
    end
end

function is_valid = dynamical_feasible(gap_list, start_node, new_node, planner_params)
    % Spatio일 때만 호출됨. 시작점은 전역 start_node 기준.
    start_gap = gap_list(start_node);
    new_gap   = gap_list(new_node);

    planning_resolution = planner_params.PlanningResolution; % (= prediction_dt)
    ad_max = planner_params.MaxLatAccel;                      % (= boundary.ad_max)

    dt = planning_resolution * max(1, (new_gap.time_idx - start_gap.time_idx));
    delta_d = abs(new_gap.d - start_gap.d)/2;
    a_required = 2*delta_d / (dt^2);

    is_valid = (a_required < ad_max);
end