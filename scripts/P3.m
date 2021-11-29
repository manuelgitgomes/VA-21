%% Exercise 1
clear, clc, close all
veloReader = velodyneFileReader('lidarData_ConstructionRoad.pcap', 'HDL32E');
ptCloud = readFrame(veloReader);
%Definir os limites da zona a representar
xlimits = [-25 45]; %em metros
ylimits = [-25 45];
zlimits = [-20 20];
lidarViewer = pcplayer(xlimits, ylimits, zlimits); 
%Definir os labels dos eixos
xlabel(lidarViewer.Axes, 'X (m)');
ylabel(lidarViewer.Axes, 'Y (m)');
zlabel(lidarViewer.Axes, 'Z (m)');
%Visualizar a nuvem de pontos
view(lidarViewer, ptCloud);

%% Exercise 2
clear, clc, close all
veloReader = velodyneFileReader('lidarData_ConstructionRoad.pcap', 'HDL32E');
ptCloud = readFrame(veloReader);
%Definir os limites da zona a representar
xlimits = [-25 45]; %em metros
ylimits = [-25 45];
zlimits = [-20 20];
lidarViewer = pcplayer(xlimits, ylimits, zlimits); 
%Definir os labels dos eixos
xlabel(lidarViewer.Axes, 'X (m)');
ylabel(lidarViewer.Axes, 'Y (m)');
zlabel(lidarViewer.Axes, 'Z (m)');
%Define vehicle dimensions
vehicleDims = vehicleDimensions(4.7, 1.8, 1.4);
tol = 1.1;
% With these dimensions, specify the limit
limits = tol * [-vehicleDims.Length/2 vehicleDims.Length/2;
-vehicleDims.Width/2 vehicleDims.Width/2;
-vehicleDims.Height 0];
% Create a mask for EgoPoints
points = struct();
points.EgoPoints = ptCloud.Location(:, :, 1) > limits(1,1) ...
        & ptCloud.Location(:, :, 1) < limits(1,2) ...
        & ptCloud.Location(:, :, 2) > limits(2,1) ...
        & ptCloud.Location(:, :, 2) < limits(2,2) ...
        & ptCloud.Location(:, :, 3) > limits(3,1) ...
        & ptCloud.Location(:, :, 3) < limits(3,2);
% Create and label colors
colorLabels = [0       0.4470  0.7410;
               0.4660  0.6740  0.1880;
               0.9290  0.6940  0.1250;
               0.6350  0.0780  0.1840];
colors.Unlabeled = 1; %Indexar as cores
colors.Ground = 2;
colors.Ego = 3;
colors.Obstacle = 4;
% Create a colormap
colormap(lidarViewer.Axes, colorLabels);
scanSize = size(ptCloud.Location);
scanSize = scanSize(1:2);
colormapValues = ones(scanSize, 'like', ...
ptCloud.Location)*colors.Unlabeled;
% Define EgoPoints with a certain color
colormapValues(points.EgoPoints) = colors.Ego;
view(lidarViewer, ptCloud.Location, colormapValues)
% Exercicio 3
% Segment ground
points.GroundPoints = segmentGroundFromLidarData(ptCloud, ...
    'ElevationAngleDelta', 10);
% Remove Ego Points from Ground Points
points.GroundPoints = and(points.GroundPoints,not(points.EgoPoints));
% Create colormap
colormapValues(points.GroundPoints) = colors.Ground;
view(lidarViewer, ptCloud.Location, colormapValues)
nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
ptCloudSegmented = select(ptCloud, nonEgoGroundPoints,'Output','full');
points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, [0 0 0], 40);
colormapValues(points.ObstaclePoints) = colors.Obstacle;
view(lidarViewer, ptCloud.Location, colormapValues)

%% Exercise 5
clear, clc, close all
veloReader = velodyneFileReader('lidarData_ConstructionRoad.pcap', 'HDL32E');

%Definir os limites da zona a representar
xlimits = [-25 45]; %em metros
ylimits = [-25 45];
zlimits = [-20 20];
lidarViewer = pcplayer(xlimits, ylimits, zlimits); 
lidarViewer2 = pcplayer(xlimits, ylimits, zlimits); 

% Create and label colors
colorLabels = [0       0.4470  0.7410;
               0.4660  0.6740  0.1880;
               0.9290  0.6940  0.1250;
               0.6350  0.0780  0.1840];
colors.Unlabeled = 1; %Indexar as cores
colors.Ground = 2;
colors.Ego = 3;
colors.Obstacle = 4;

% Create a colormap
colormap(lidarViewer.Axes, colorLabels);

%Define vehicle dimensions
vehicleDims = vehicleDimensions(4.7, 1.8, 1.4);
tol = 1.1;

% With these dimensions, specify the limit
limits = tol * [-vehicleDims.Length/2 vehicleDims.Length/2;
-vehicleDims.Width/2 vehicleDims.Width/2;
-vehicleDims.Height 0];
stopTime = veloReader.StartTime + seconds(30);

minNumPoints = 50;

while hasFrame(veloReader) && veloReader.CurrentTime < stopTime
    ptCloud = readFrame(veloReader);
    % Create a mask for EgoPoints
    points = struct();
    points.EgoPoints = ptCloud.Location(:, :, 1) > limits(1,1) ...
            & ptCloud.Location(:, :, 1) < limits(1,2) ...
            & ptCloud.Location(:, :, 2) > limits(2,1) ...
            & ptCloud.Location(:, :, 2) < limits(2,2) ...
            & ptCloud.Location(:, :, 3) > limits(3,1) ...
            & ptCloud.Location(:, :, 3) < limits(3,2);
    scanSize = size(ptCloud.Location);
    scanSize = scanSize(1:2);
    colormapValues = ones(scanSize, 'like', ...
    ptCloud.Location)*colors.Unlabeled;

    % Define EgoPoints with a certain color
    colormapValues(points.EgoPoints) = colors.Ego;
    %view(lidarViewer, ptCloud.Location, colormapValues)
    % Exercicio 3

    % Segment ground
    points.GroundPoints = segmentGroundFromLidarData(ptCloud, ...
        'ElevationAngleDelta', 10);
    
    % Remove Ego Points from Ground Points
    points.GroundPoints = and(points.GroundPoints,not(points.EgoPoints));

    % Create colormap
    colormapValues(points.GroundPoints) = colors.Ground;
    %view(lidarViewer, ptCloud.Location, colormapValues)
    nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
    ptCloudSegmented = select(ptCloud, nonEgoGroundPoints,'Output','full');
    points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, [0 0 0], 40);
    colormapValues(points.ObstaclePoints) = colors.Obstacle;
    figure(1)
    view(lidarViewer, ptCloud.Location, colormapValues)

    figure(2)
    
    [labels, numClusters] = segmentLidarData(ptCloudSegmented, 1, 180, 'NumClusterPoints', minNumPoints);
    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloudSegmented, idxValidPoints);
    view(lidarViewer2, segmentedPtCloud.Location, labelColorIndex)
end