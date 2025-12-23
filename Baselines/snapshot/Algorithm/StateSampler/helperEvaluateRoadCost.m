function Gaps = helperEvaluateRoadCost(numGaps, Gaps)

for i = 1:numGaps
    Gaps(i).road_cost = 1 - Gaps(i).l/100;
end

end