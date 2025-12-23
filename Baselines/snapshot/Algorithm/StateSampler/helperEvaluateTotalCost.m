function Gaps = helperEvaluateTotalCost(numGaps, Gaps)

weight_road = 2.5;
weight_traffic = 1.5;
weight_arrival = 0.2;

for i = 1:numGaps
    Gaps(i).total_cost = weight_road*Gaps(i).road_cost + ...
                         weight_traffic*Gaps(i).traffic_cost + ...
                         weight_arrival*Gaps(i).arrival_cost;
end