function refPathOut = helperGetReferencePath()
persistent refPath
if isempty(refPath)
waypoints = [-43.7 52.9; 13.7 40.1; 57.1 14.5; 42.3 -63; 115.9 -120.8; 214.2 -152.7];
    refPath = referencePathFrenet(waypoints);
end
refPathOut = refPath;
end