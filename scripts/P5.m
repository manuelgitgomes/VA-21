%% Exercise 1
clear, clc, close all
vFile = 'caltech_cordova1.avi';
v = VideoReader(vFile);
v.CurrentTime = 0;
vidFrame = readFrame(v);
img = imshow(vidFrame);
tt=title(sprintf('Current Time = %.3f sec', v.CurrentTime));
pause % press a key to continue
while v.hasFrame
    vidFrame = readFrame(v) ;
    img.CData = vidFrame ;
    str = sprintf('Current Time = %.3f sec', v.CurrentTime);
    tt.String = str;
    pause(1/v.FrameRate)
end

%% Exercise 2
clear, clc, close all
focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] optical center in pixel coordinates
imageSize      = [480, 640];           % [nrows, mcols]
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
height = 2.1798;    % mounting height in meters from the ground
pitch  = 14;        % pitch of the camera in degrees
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);
videoName = 'caltech_cordova1.avi';
videoReader = VideoReader(videoName);
timeStamp = 0.06667;                   % time from the beginning of the video
videoReader.CurrentTime = timeStamp;   % point to the chosen frame
frame = readFrame(videoReader); % read frame at timeStamp seconds
imshow(frame) % display frame
% Using vehicle coordinates, define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 6;  % all other distance quantities are also in meters
bottomOffset      = 3;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
birdsEyeImage = transformImage(birdsEyeConfig, frame);
figure
imshow(birdsEyeImage)
% Convert to grayscale
birdsEyeImage = im2gray(birdsEyeImage);

% Lane marker segmentation ROI in world units
vehicleROI = outView - [-1, 2, -3, 3]; % look 3 meters to left and right, and 4 meters ahead of the sensor
approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters

% Detect lane features
laneSensitivity = 0.25;
birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, approxLaneMarkerWidthVehicle,...
    'ROI', vehicleROI, 'Sensitivity', laneSensitivity);

figure
imshow(birdsEyeViewBW)
% Obtain lane candidate points in vehicle coordinates
[imageX, imageY] = find(birdsEyeViewBW);
xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
maxLanes      = 2; % look for maximum of two lane markers
boundaryWidth = 3*approxLaneMarkerWidthVehicle; % expand boundary width

[boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
    'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);

% Establish criteria for rejecting boundaries based on their length
maxPossibleXLength = diff(vehicleROI(1:2));
minXLength         = maxPossibleXLength * 0.60; % establish a threshold

% Find short boundaries
if( numel(boundaries) > 0 )
    isOfMinLength = false(1, numel(boundaries));
    for i = 1 : numel(boundaries)
        if(diff(boundaries(i).XExtent) > minXLength)
            isOfMinLength(i) = true;
        end
    end
else
    isOfMinLength = false;
end

% To compute the maximum strength, assume all image pixels within the ROI
% are lane candidate points
birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));

% Convert the image points to vehicle points
vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

% Find the maximum number of unique x-axis locations possible for any lane
% boundary
maxPointsInOneLane = numel(unique(single((vehiclePoints(:,1)))));

% Set the maximum length of a lane boundary to the ROI length
maxLaneLength = diff(vehicleROI(1:2));

% Compute the maximum possible lane strength for this image size/ROI size
% specification
maxStrength   = maxPointsInOneLane/maxLaneLength;

% Reject short and weak boundaries
idx = 0;
strongBoundaries = parabolicLaneBoundary(zeros(nnz(isOfMinLength), 3));
for i = 1 : size(isOfMinLength,2)
    if( isOfMinLength(i) == 1 )
        if( boundaries(i).Strength > 0.4*maxStrength )
            idx = idx + 1;
            strongBoundaries(idx) = boundaries(i);
        end
    end
end

% Classify lane marker type when boundaryPoints are not empty
if isempty(boundaryPoints)
    strongBoundaries = repmat(strongBoundaries,1,2);
    strongBoundaries(1) = parabolicLaneBoundary(zeros(1,3));
    strongBoundaries(2) = parabolicLaneBoundary(zeros(1,3));
else
    strongBoundaries = classifyLaneTypes(strongBoundaries, boundaryPoints);
end

distancesToBoundaries = coder.nullcopy(ones(size(strongBoundaries,2),1));

% Find ego lanes
xOffset    = 0;   %  0 meters from the sensor
for i = 1 : size(strongBoundaries, 2)
    distancesToBoundaries(i) = strongBoundaries(i).computeBoundaryModel(xOffset);
end

% Find candidate ego boundaries
distancesToLeftBoundary = distancesToBoundaries>0;
if (numel(distancesToBoundaries(distancesToLeftBoundary)))
    minLeftDistance = min(distancesToBoundaries(distancesToLeftBoundary));
else
    minLeftDistance = 0;
end

distancesToRightBoundary = (distancesToBoundaries <= 0);
if( numel(distancesToBoundaries(distancesToRightBoundary)))
    minRightDistance = max(distancesToBoundaries(distancesToRightBoundary));
else
    minRightDistance = 0;
end

% Find left ego boundary
if (minLeftDistance ~= 0)
    leftEgoBoundaryIndex  = distancesToBoundaries == minLeftDistance;
    leftEgoBoundary = parabolicLaneBoundary(zeros(nnz(leftEgoBoundaryIndex), 3));
    idx = 0;
    for i = 1 : size(leftEgoBoundaryIndex, 1)
        if( leftEgoBoundaryIndex(i) == 1)
            idx = idx + 1;
            leftEgoBoundary(idx) = strongBoundaries(i);
        end
    end
else
    leftEgoBoundary = parabolicLaneBoundary.empty();
end

% Find right ego boundary
if (minRightDistance ~= 0)
    rightEgoBoundaryIndex = distancesToBoundaries == minRightDistance;
    rightEgoBoundary = parabolicLaneBoundary(zeros(nnz(rightEgoBoundaryIndex), 3));
    idx = 0;
    for i = 1 : size(rightEgoBoundaryIndex, 1)
        if( rightEgoBoundaryIndex(i) == 1)
            idx = idx + 1;
            rightEgoBoundary(idx) = strongBoundaries(i);
        end
    end
else
    rightEgoBoundary = parabolicLaneBoundary.empty();
end

xVehiclePoints = bottomOffset:distAheadOfSensor;
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeWithEgoLane, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');

frameWithEgoLane = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
frameWithEgoLane = insertLaneBoundary(frameWithEgoLane, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');

figure
subplot('Position', [0, 0, 0.5, 1.0]) % [left, bottom, width, height] in normalized units
imshow(birdsEyeWithEgoLane)
subplot('Position', [0.5, 0, 0.5, 1.0])
imshow(frameWithEgoLane)
%% Extra
figure(3)
hold on
for i=1:numel(boundaryPoints)
    lane = boundaryPoints{i};
    xylane = imageToVehicle(birdsEyeConfig, lane);
    plot(xylane(:,1), xylane(:,2), 'o')
end

%% Exercise 3
clear, clc, close all
focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] optical center in pixel coordinates
imageSize      = [480, 640];           % [nrows, mcols]
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
height = 2.1798;    % mounting height in meters from the ground
pitch  = 14;        % pitch of the camera in degrees
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);
videoName = 'caltech_cordova1.avi';
videoReader = VideoReader(videoName);
timeStamp = 0.06667;                   % time from the beginning of the video
videoReader.CurrentTime = timeStamp;   % point to the chosen frame
frame = readFrame(videoReader); % read frame at timeStamp seconds
imshow(frame) % display frame
% Using vehicle coordinates, define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 6;  % all other distance quantities are also in meters
bottomOffset      = 3;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
birdsEyeImage = transformImage(birdsEyeConfig, frame);
figure
imshow(birdsEyeImage)
% Convert to grayscale
birdsEyeImage = im2gray(birdsEyeImage);

% Lane marker segmentation ROI in world units
vehicleROI = outView - [-1, 2, -3, 3]; % look 3 meters to left and right, and 4 meters ahead of the sensor
approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters

% Detect lane features
laneSensitivity = 0.25;
birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, approxLaneMarkerWidthVehicle,...
    'ROI', vehicleROI, 'Sensitivity', laneSensitivity);

figure
imshow(birdsEyeViewBW)
% Obtain lane candidate points in vehicle coordinates
[imageX, imageY] = find(birdsEyeViewBW);
xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
maxLanes      = 2; % look for maximum of two lane markers
boundaryWidth = 3*approxLaneMarkerWidthVehicle; % expand boundary width

[boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
    'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);

% Establish criteria for rejecting boundaries based on their length
maxPossibleXLength = diff(vehicleROI(1:2));
minXLength         = maxPossibleXLength * 0.60; % establish a threshold

% Find short boundaries
if( numel(boundaries) > 0 )
    isOfMinLength = false(1, numel(boundaries));
    for i = 1 : numel(boundaries)
        if(diff(boundaries(i).XExtent) > minXLength)
            isOfMinLength(i) = true;
        end
    end
else
    isOfMinLength = false;
end

% To compute the maximum strength, assume all image pixels within the ROI
% are lane candidate points
birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));

% Convert the image points to vehicle points
vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

% Find the maximum number of unique x-axis locations possible for any lane
% boundary
maxPointsInOneLane = numel(unique(single((vehiclePoints(:,1)))));

% Set the maximum length of a lane boundary to the ROI length
maxLaneLength = diff(vehicleROI(1:2));

% Compute the maximum possible lane strength for this image size/ROI size
% specification
maxStrength   = maxPointsInOneLane/maxLaneLength;

% Reject short and weak boundaries
idx = 0;
strongBoundaries = parabolicLaneBoundary(zeros(nnz(isOfMinLength), 3));
for i = 1 : size(isOfMinLength,2)
    if( isOfMinLength(i) == 1 )
        if( boundaries(i).Strength > 0.4*maxStrength )
            idx = idx + 1;
            strongBoundaries(idx) = boundaries(i);
        end
    end
end

% Classify lane marker type when boundaryPoints are not empty
if isempty(boundaryPoints)
    strongBoundaries = repmat(strongBoundaries,1,2);
    strongBoundaries(1) = parabolicLaneBoundary(zeros(1,3));
    strongBoundaries(2) = parabolicLaneBoundary(zeros(1,3));
else
    strongBoundaries = classifyLaneTypes(strongBoundaries, boundaryPoints);
end

distancesToBoundaries = coder.nullcopy(ones(size(strongBoundaries,2),1));

% Find ego lanes
xOffset    = 0;   %  0 meters from the sensor
for i = 1 : size(strongBoundaries, 2)
    distancesToBoundaries(i) = strongBoundaries(i).computeBoundaryModel(xOffset);
end

% Find candidate ego boundaries
distancesToLeftBoundary = distancesToBoundaries>0;
if (numel(distancesToBoundaries(distancesToLeftBoundary)))
    minLeftDistance = min(distancesToBoundaries(distancesToLeftBoundary));
else
    minLeftDistance = 0;
end

distancesToRightBoundary = (distancesToBoundaries <= 0);
if( numel(distancesToBoundaries(distancesToRightBoundary)))
    minRightDistance = max(distancesToBoundaries(distancesToRightBoundary));
else
    minRightDistance = 0;
end

% Find left ego boundary
if (minLeftDistance ~= 0)
    leftEgoBoundaryIndex  = distancesToBoundaries == minLeftDistance;
    leftEgoBoundary = parabolicLaneBoundary(zeros(nnz(leftEgoBoundaryIndex), 3));
    idx = 0;
    for i = 1 : size(leftEgoBoundaryIndex, 1)
        if( leftEgoBoundaryIndex(i) == 1)
            idx = idx + 1;
            leftEgoBoundary(idx) = strongBoundaries(i);
        end
    end
else
    leftEgoBoundary = parabolicLaneBoundary.empty();
end

% Find right ego boundary
if (minRightDistance ~= 0)
    rightEgoBoundaryIndex = distancesToBoundaries == minRightDistance;
    rightEgoBoundary = parabolicLaneBoundary(zeros(nnz(rightEgoBoundaryIndex), 3));
    idx = 0;
    for i = 1 : size(rightEgoBoundaryIndex, 1)
        if( rightEgoBoundaryIndex(i) == 1)
            idx = idx + 1;
            rightEgoBoundary(idx) = strongBoundaries(i);
        end
    end
else
    rightEgoBoundary = parabolicLaneBoundary.empty();
end

xVehiclePoints = bottomOffset:distAheadOfSensor;
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeWithEgoLane, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');

frameWithEgoLane = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
frameWithEgoLane = insertLaneBoundary(frameWithEgoLane, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');

figure
subplot('Position', [0, 0, 0.5, 1.0]) % [left, bottom, width, height] in normalized units
imshow(birdsEyeWithEgoLane)
subplot('Position', [0.5, 0, 0.5, 1.0])
imshow(frameWithEgoLane)