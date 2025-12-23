function Gaps = helperEvaluateArrivalCost(numGaps, Gaps, ST, MapInfo)

num_topology = length(find(ST(1,:) ~= 0));
s_f = zeros(1, numGaps);
s_r = zeros(1, numGaps);
LaneWidth = MapInfo.LaneWidth(1);

for i = 1:numGaps
    s_f(i) = min(50, Gaps(i).s + Gaps(i).l / 2);
    s_r(i) = max(-50, Gaps(i).s - Gaps(i).l / 2);
end

C_arrival = 2*ones(1,numGaps);
C_arrival(ST(1,1)) = 0;
Gaps(ST(1,1)).arrival_cost = 0;
egoGap = Gaps(ST(1,1));

for i = 1 : num_topology
    gap_A = ST(1,i);
    gap_B = ST(2,i);

    A_f = s_f(gap_A);
    A_r = s_r(gap_A);
    B_f = s_f(gap_B);
    B_r = s_r(gap_B);


    C_lon = 1 - ((min(A_f, B_f) - max(A_r, B_r)) / Gaps(gap_A).l);
    C_lat = 0.1 * LaneWidth * abs(Gaps(gap_A).lane_idx - Gaps(gap_B).lane_idx);
    C_A2B = C_lon + C_lat;

    if abs(Gaps(gap_B).lane_idx - egoGap.lane_idx) <= 1
        C_temp = C_A2B + C_arrival(gap_A);
        if C_arrival(gap_B) > C_temp
            C_arrival(gap_B) = C_temp;
            Gaps(gap_B).arrival_cost = C_temp;
        end
    else
        Gaps(gap_B).arrival_cost = C_arrival(gap_B) + 10.0;
    end
end