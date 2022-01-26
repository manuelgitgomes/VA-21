function refPathOut = helperGetReferencePath()
persistent refPath
if isempty(refPath)
    waypoints = [10.1 0 0;
    31.4 0 0;
    39.9 0 0;
    48.6 0 0;
    62.5 0 0;
    73.5 0 0;
    83.6 0 0];
    waypoints(3,:) = [];
    refPath = referencePathFrenet(waypoints);
end
refPathOut = refPath;
end