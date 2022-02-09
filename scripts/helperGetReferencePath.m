function refPathOut = helperGetReferencePath()
persistent refPath
if isempty(refPath)
% roadCenters = [-102.7 20.8 0;
%     -77.4 18.6 0;
%     -61.4 13.5 0;
%     -49.7 0.8 0;
%     -45.8 -21.5 0;
%     -27.6 -27.3 0;
%     -21.3 -46.1 0;
%     -20.8 -59.9 0];
roadCenters = [-43.7 52.9 0;
    13.7 40.1 0;
    57.1 14.5 0;
    42.3 -63 0;
    115.9 -120.8 0;
    151 -131.2 0];
waypoints = roadCenters(:, 1:2);
% waypoints = [-102.7 20.8; -77.4 18.6; -61.4 13.5; -49.7 0.8; -45.8 21.5; -27.6 27.3; -21.3 -46.1; -20.8 -59.9];
    refPath = referencePathFrenet(waypoints);
end
refPathOut = refPath;
end