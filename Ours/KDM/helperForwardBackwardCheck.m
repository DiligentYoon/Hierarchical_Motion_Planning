function [ok, vlo_next, vhi_next] = helperForwardBackwardCheck( ...
    Gaps, start_node, u, v, vlo_u, vhi_u, ...
    a_min, a_max, vminG, vmaxG, Planner_params, ...
    B_lo, B_hi, hasB, ev, lv)

%#codegen
    dt0 = Planner_params.PlanningResolution;
    dt = dt0;

    % 1) forward reachable speed interval
    vlo = max(vminG, vlo_u + a_min*dt);
    vhi = min(vmaxG, vhi_u + a_max*dt);
    if vlo > vhi
        ok = false; vlo_next = 0; vhi_next = 0; return;
    end

    % 2) next node local band 교집합
    [vBand_lo, vBand_hi, vBand_ok] = getLocalBand(Gaps, start_node, v, vminG, vmaxG);
    if vBand_ok
        vlo = max(vlo, vBand_lo);
        vhi = min(vhi, vBand_hi);
        if vlo > vhi
            ok = false; vlo_next = 0; vhi_next = 0; return;
        end
    else
        ok = false; vlo_next = 0; vhi_next = 0; return;
    end

    % 3) backward-feasible band 교집합
    if hasB(v,ev,lv)
        vlo = max(vlo, B_lo(v,ev,lv));
        vhi = min(vhi, B_hi(v,ev,lv));
        if vlo > vhi
            ok = false; vlo_next = 0; vhi_next = 0; return;
        end
    else
        ok = false; vlo_next = 0; vhi_next = 0; return;
    end

    % 4) s interval reachability
    su_min = Gaps(u).s - Gaps(u).l/2;
    su_max = Gaps(u).s + Gaps(u).l/2;
    sv_min = Gaps(v).s - Gaps(v).l/2;
    sv_max = Gaps(v).s + Gaps(v).l/2;

    u_pre_lo = max(vminG, vlo - a_max*dt);
    u_pre_hi = min(vmaxG, vhi - a_min*dt);
    u_lo = max(vlo_u, u_pre_lo);
    u_hi = min(vhi_u, u_pre_hi);

    s_reach_min = su_min + u_lo*dt + 0.5*a_min*dt^2;
    s_reach_max = su_max + u_hi*dt + 0.5*a_max*dt^2;

    if (s_reach_max < sv_min) || (s_reach_min > sv_max)
        ok = false; vlo_next = 0; vhi_next = 0; return;
    end

    ok = true;
    vlo_next = vlo;
    vhi_next = vhi;
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