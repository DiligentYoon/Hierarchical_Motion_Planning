function [B_lo, B_hi, hasB] = helperComputeBackwardFeasibleBand( ...
    Gaps, neighbors_mat, neighbors_len, S, start_node, end_nodes, ...
    a_min, a_max, vminG, vmaxG, Planner_params)

    N = numel(Gaps);
    FALSE = 1; TRUE = 2;
    maxDeg = 10;

    B_lo =  inf(N,2,2);
    B_hi = -inf(N,2,2);
    hasB = false(N,2,2);

    dt0 = Planner_params.PlanningResolution;

    for i = 1:numel(end_nodes)
        n = end_nodes(i);
        if n == 0, continue; end
        [vloT, vhiT, okT] = getLocalBand(Gaps, start_node, n, vminG, vmaxG);
        if ~okT, continue; end
        for eix = 1:2
            for lix = 1:2
                % End nodes의 모든 (e,l) 상태에 대해 동일한 값 적용
                B_lo(n, eix, lix) = vloT;
                B_hi(n, eix, lix) = vhiT;
                hasB(n, eix, lix) = true;
            end
        end
    end

    % Back 연산을 위한 내림차순 정리
    timeIdx = zeros(N, 1);
    for n = 1:N
        timeIdx(n) = Gaps(n).time_idx;
    end
    [~, order] = sort(timeIdx, 'descend');


    % Backward DP
    for oi = 1:N
        u = order(oi);
        tu = Gaps(u).time_idx;
        len_u = min(neighbors_len(u), maxDeg);
        if len_u == 0, continue; end
        
        % 미리 구했었던 Local Band 중, u가 속한 차선을 기준으로 가져옴.
        [uBand_lo, uBand_hi, uBand_ok] = getLocalBand(Gaps, start_node, u, vminG, vmaxG);
        
        for eu = 1:2
            for lu = 1:2
                lastIsSp_u = (lu == TRUE);
                everSP_u   = (eu == TRUE);

                best_lo = inf;
                best_hi = -inf;
                found = false;

                for k = 1:len_u
                    % u -> v Edge
                    v = neighbors_mat(u, k);
                    if v == 0, continue; end
                    
                    tv = Gaps(v).time_idx;
                    if tv <= tu
                        % Temporal 가정에 맞지 않음
                        continue;
                    end
                    
                    % NOTE: Lane Change Rule 검사. 위 Spatio 조건에서 막아버리기 때문에 걸릴일 없을 것으로 예상
                    edgeIsSp = S(u, v);
                    if lastIsSp_u && edgeIsSp
                        continue;
                    end

                    if edgeIsSp
                        if everSP_u
                            continue;
                        end
                        if ~dynamical_feasible(Gaps, start_node, v, Planner_params)
                            continue;
                        end
                    end

                    lv = FALSE; if edgeIsSp, lv = TRUE; end
                    ev = FALSE; if (everSP_u || edgeIsSp), ev = TRUE; end

                    if ~hasB(v, ev, lv)
                        % successor Node가 Feasible Bounds를 가지지 못한다면 탐색 가치가 없음.
                        continue;
                    end
                    
                    % 동역학 역산 with Feasible Bounds
                    dt = dt0 * max(1, (tv - tu));
                    lo2 = B_lo(v, ev, lv);
                    hi2 = B_hi(v, ev, lv);

                    pre_lo = max(vminG, lo2 - a_max*dt);
                    pre_hi = min(vmaxG, hi2 - a_min*dt);

                    if pre_lo <= pre_hi
                        found = true;
                        best_lo = min(best_lo, pre_lo);
                        best_hi = max(best_hi, pre_hi);
                    end
                end

                if found
                    % Feasible Bound가 존재
                    if uBand_ok
                        % uBand가 존재 -> 교집합 연산
                        best_lo = max(best_lo, uBand_lo);
                        best_hi = min(best_hi, uBand_hi);
                        if best_lo > best_hi
                            continue;
                        end
                    end
                    B_lo(u, eu, lu) = best_lo;
                    B_hi(u, eu, lu) = best_hi;
                    hasB(u, eu, lu) = true;
                end
            end
        end 
    end

    % spatio edge bacward DP
    maxIter = 2;
    for it = 1:maxIter
        changed = false;
        for u = 1:N
            tu = Gaps(u).time_idx;
            len_u = min(neighbors_len(u), maxDeg);
            if len_u == 0, continue; end

            [uBand_lo, uBand_hi, uBand_ok] = getLocalBand(Gaps, start_node, u, vminG, vmaxG);
            
            for eu = 1:2
                for lu = 1:2
                    lastIsSp_u = (lu == TRUE);
                    everSP_u   = (eu == TRUE);

                    best_lo = B_lo(u, eu, lu);
                    best_hi = B_hi(u, eu, lu);
                    found = hasB(u, eu, lu);

                    for k = 1:len_u
                        v = neighbors_mat(u, k);
                        if v == 0, continue; end
                        tv = Gaps(v).time_idx;
                        if tv ~= tu
                            % Spatio 관계는 time index가 동일해야 함
                            continue;
                        end

                        edgeIsSp = S(u, v);
                        if ~edgeIsSp
                            % 동일 time에서는 spatio만 전파
                            continue;
                        end
                        if lastIsSp_u
                            % 직전에 차선 변경을 수행한 상황
                            continue;
                        end
                        if everSP_u
                            % 이전에 차선 변경을 수행한 상황
                            continue; 
                        end
                        if ~dynamical_feasible(Gaps, start_node, v, Planner_params)
                            % 횡방향 동특성 제약
                            continue;
                        end

                        lv = TRUE;
                        ev = TRUE;

                        if ~hasB(v, ev, lv)
                            % successor 존재안하는 상황
                            continue;
                        end

                        dt = dt0;
                        lo2 = B_lo(v,ev,lv);
                        hi2 = B_hi(v,ev,lv);

                        pre_lo = lo2 - a_max*dt;
                        pre_hi = hi2 - a_min*dt;

                        pre_lo = max(pre_lo, vminG);
                        pre_hi = min(pre_hi, vmaxG);

                        if pre_lo <= pre_hi
                            found = true;
                            best_lo = min(best_lo, pre_lo);
                            best_hi = max(best_hi, pre_hi);
                        end
                    end

                    if found
                        if uBand_ok
                            best_lo2 = max(best_lo, uBand_lo);
                            best_hi2 = min(best_hi, uBand_hi);
                            if best_lo2 <= best_hi2
                                if ~hasB(u,eu,lu) || best_lo2 < B_lo(u,eu,lu) || best_hi2 > B_hi(u,eu,lu)
                                    B_lo(u,eu,lu) = best_lo2;
                                    B_hi(u,eu,lu) = best_hi2;
                                    hasB(u,eu,lu) = true;
                                    changed = true;
                                end
                            end
                        end
                    end

                end
            end
        end
        if ~changed
            break;
        end
    end
end


function [vlo, vhi, ok] = getLocalBand(Gaps, start_node, n, vminG, vmaxG)
    % start node를 기준으로, 특정 node n에 대한 Local Band를 구함
%#codegen
    row = laneRowFromD(Gaps, start_node, n);
    band = Gaps(n).vel_range(row, :); % 1x2
    if (band(1) == 0 && band(2) == 0)
        ok = false; vlo = 0; vhi = 0; return;
    end
    vlo = band(1);
    vhi = band(2);
    ok = (vlo <= vhi);
end

function row = laneRowFromD(Gaps, start_node, n)
%#codegen
    d0 = Gaps(start_node).d;
    dn = Gaps(n).d;
    epsd = 1e-6;
    if abs(dn - d0) < epsd
        row = 2;   % ego lane
    elseif dn < d0
        row = 1;   % right
    else
        row = 3;   % left
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