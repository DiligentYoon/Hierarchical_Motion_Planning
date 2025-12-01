function SPset_with_connectable = helperCreateConnectableSet(InitStruct_LC, ...
                                                             InitStruct_LK, ...
                                                             SPset)
                                                             
    N_t = 9;
    SPset_with_connectable = SPset;

    %% Lane Change Connectable Set
    for i = 1:N_t-1
        cc_t = InitStruct_LC;
        f_t = SPset_with_connectable.F_t(i);
        % Default Value
        Gaps = [f_t.G_t(1, 1:f_t.num_gaps_lane(1)), ...
                f_t.G_t(2, 1:f_t.num_gaps_lane(2)), ...
                f_t.G_t(3, 1:f_t.num_gaps_lane(3))];
        time_idx = i;
        num_gaps = f_t.num_gaps;
        num_gaps_lane = [f_t.num_gaps_lane(1), ...
                         f_t.num_gaps_lane(2), ...
                         f_t.num_gaps_lane(3)];
        % reachable matrix
        reachable_mtrx = reachable(f_t, Gaps);
        % Topology Graph
        [lc_topology_graph, ...
         lc_topology_global_graph] = set_LC_connectable_set(reachable_mtrx, Gaps);
        num_node = size(lc_topology_graph, 2); 
        % LC Connectable Set
        cc_t.time_idx = time_idx;
        cc_t.num_gaps = num_gaps;
        cc_t.num_gaps_lane = num_gaps_lane;
        cc_t.num_nodes = num_node;
        cc_t.reachable_mtrx(1:num_gaps, 1:num_gaps) = reachable_mtrx;
        cc_t.spatio_topology_graph(:, 1:num_node) = lc_topology_graph;
        cc_t.spatio_topology_global_graph(:, 1:num_node) = lc_topology_global_graph;
        cc_t.gaps(1:num_gaps) = Gaps;
        % Setting to SPset
        SPset_with_connectable.Cc_t(i) = cc_t;
    end

    %% Lane Keeping Connectable Set
    for i = 1:N_t-2
        ck_t = InitStruct_LK;
        cc_t1 = SPset_with_connectable.Cc_t(i);
        cc_t2 = SPset_with_connectable.Cc_t(i+1);
        [lk_topology_global_graph, ...
         num_temporal_set] =    set_LK_connectable_set(cc_t1, cc_t2, ...
                                                       ck_t.temporal_topology_global_graph, ...
                                                       ck_t.overlap_threshold);
        ck_t.time_idx = i;
        ck_t.current_spatio_set = cc_t1;
        ck_t.next_spatio_set = cc_t2;
        ck_t.num_nodes = num_temporal_set;
        ck_t.temporal_topology_global_graph = lk_topology_global_graph;
        SPset_with_connectable.Ck_t(i) = ck_t;
    end
end



function reachable_mtrx = reachable(f_t, Gaps)
    num_gaps = f_t.num_gaps;
    reachable_mtrx = zeros(num_gaps, num_gaps);

    for i = 1:num_gaps
        for j = 1:num_gaps
            if i == j 
                % 자기 자신과는 연결되지 않음
                continue; 
            end

            is_reachable = false; 
            is_adjacent_lane = abs(Gaps(i).lane_idx - Gaps(j).lane_idx) == 1;
            is_overlapping = abs(Gaps(i).s - Gaps(j).s) < (Gaps(i).l + Gaps(j).l)/2;
            has_front_gap = (Gaps(i).s + Gaps(i).l/2 > 0) && (Gaps(j).s + Gaps(j).l/2 > 0);
            
            if is_adjacent_lane && is_overlapping && has_front_gap
                is_reachable = true;
            end
            
            reachable_mtrx(i,j) = is_reachable;
        end
    end
end


function [lc_topology_graph, ...
          lc_topology_global_graph] = set_LC_connectable_set(reachable_mtrx, ...
                                                             gaps)
    S = []; % Local Index 시작 노드
    T = []; % Local Index 끝 노드
    S_g = []; % Global ID 시작 노드
    T_g = []; % Global ID 끝 노드

    % reachable_mtrx를 순회하여 모든 'true' 연결을 엣지로 추가
    num_gaps = size(reachable_mtrx, 1);
    for i = 1:num_gaps
        for j = 1:num_gaps
            % 중복 Node 연결 포함 (양방향 그래프)
            if reachable_mtrx(i, j)
                % Local Index Edge List
                S = [S, i];
                T = [T, j];
                
                % Global Index Edge List
                source_node_global = gaps(i).idx;
                target_node_global = gaps(j).idx;
                S_g = [S_g, source_node_global];
                T_g = [T_g, target_node_global];
            end
        end
    end
    lc_topology_graph = [S; T];
    lc_topology_global_graph = [S_g; T_g];
end


function [temporal_topology_global_graph, ...
          num_temporal_set] =  set_LK_connectable_set(cc_t1, ...
                                                      cc_t2, ...
                                                      dummy_graph, ...
                                                      overlap_threshold)
    % 동일 차선 조건
    % 종방향 겹침 조건 (근접성)
    % 한 스텝 앞의 Spatio Set들만 갖고 검사
    % Gap의 분할 & 병합에 따라 Temporal Edge 생성 로직 다르게 적용
    c_current = cc_t1;
    c_next = cc_t2;
    num_gap_lane_current = c_current.num_gaps_lane;
    num_gap_lane_next = c_next.num_gaps_lane;
    is_divided = num_gap_lane_current < num_gap_lane_next;
    temporal_topology_global_graph = dummy_graph;
    num_temporal_set = 0;
    for i = 1:c_current.num_gaps
        if ~isempty(c_current.gaps(i))
            current_gap = c_current.gaps(i);
            current_gap_global_id = current_gap.idx;
            current_gap_s_max = current_gap.s + current_gap.l/2;
            current_gap_s_min = current_gap.s - current_gap.l/2;
            for j = 1:c_next.num_gaps
                if ~isempty(c_next.gaps(j))
                    next_gap = c_next.gaps(j);
                    next_gap_global_id = next_gap.idx;
                    next_gap_s_max = next_gap.s + next_gap.l/2;
                    next_gap_s_min = next_gap.s - next_gap.l/2;
                    
                    gap_s_delta = next_gap.s - current_gap.s;
                    gap_overlap = min(next_gap_s_max, current_gap_s_max) - max(next_gap_s_min, current_gap_s_min);
                    gap_l_min = min(next_gap.l, current_gap.l);
                    if (current_gap.lane_idx == next_gap.lane_idx) &&...
                        (gap_overlap > gap_l_min * overlap_threshold)
                        
                        if is_divided(current_gap.lane_idx)
                            if gap_s_delta > 0
                                num_temporal_set = num_temporal_set + 1;
                                temporal_topology_global_graph(1, num_temporal_set) = current_gap_global_id;
                                temporal_topology_global_graph(2, num_temporal_set) = next_gap_global_id;
                            end
                        else
                            num_temporal_set = num_temporal_set + 1;
                            temporal_topology_global_graph(1, num_temporal_set) = current_gap_global_id;
                            temporal_topology_global_graph(2, num_temporal_set) = next_gap_global_id;
                        end
                    end
                end
            end
        end
    end
end