%% Exercise 1
clear, clc, close all

% Open image
Img = imread('highway.png');

% Defines detectors
detector = vehicleDetectorACF();

% Applies detector on img
[bboxes,scores] = detect(detector,Img);

% Inserts an annotation on vehicles detectes
Img = insertObjectAnnotation(Img,'rectangle',bboxes,scores);

% Show image
imshow(Img)

%% Exercise 3
clear, clc, close all
% Load video
vFile = 'video.mp4';
v = VideoReader(vFile);
v.CurrentTime = 0;
vidFrame = readFrame(v);
img = imshow(vidFrame);
tt=title(sprintf('Current Time = %.3f sec', v.CurrentTime));
pause % press a key to continue
thresh = 20;
% Defines detectors
detector = vehicleDetectorACF();
while v.hasFrame
    vidFrame = readFrame(v);

    
    % Applies detector on img
    [bboxes,scores] = detect(detector,vidFrame);
    
    % Conditions and defining good and bad bboxes
    mask = scores > thresh;
    gbboxes = bboxes;
    gscores = scores;
    bbboxes = bboxes;
    bscores = scores;
    gbboxes(~mask, :) = [];
    gscores(~mask, :) = [];
    bbboxes(mask, :) = [];
    bscores(mask, :) = [];

    % Inserts an annotation on vehicles detected
    if ~isempty(gscores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',gbboxes,gscores,'Color', 'Green');
    end

    if ~isempty(bscores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',bbboxes,bscores,'Color', 'Red');
    end

    img.CData = vidFrame ;
    str = sprintf('Current Time = %.3f sec', v.CurrentTime);
    tt.String = str;
    pause(1/v.FrameRate)
end

%% Exercise 4
clear, clc, close all
% Load video
vFile = 'video.mp4';
v = VideoReader(vFile);
v.CurrentTime = 0;
vidFrame = readFrame(v);
img = imshow(vidFrame);
tt=title(sprintf('Current Time = %.3f sec', v.CurrentTime));
pause % press a key to continue
thresh = 20;
% Defines detectors
detector = peopleDetectorACF();
while v.hasFrame
    vidFrame = readFrame(v);

    
    % Applies detector on img
    [bboxes,scores] = detect(detector,vidFrame);

    % Inserts an annotation on people detected

    if ~isempty(scores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',bboxes,scores);
    end

    img.CData = vidFrame ;
    str = sprintf('Current Time = %.3f sec', v.CurrentTime);
    tt.String = str;
    pause(1/v.FrameRate)
end

%% Exercise 5
clear, clc, close all
% Load video
vFile = 'video.mp4';
v = VideoReader(vFile);
v.CurrentTime = 0;
vidFrame = readFrame(v);
img = imshow(vidFrame);
tt=title(sprintf('Current Time = %.3f sec', v.CurrentTime));
thresh = 0.8;
% Defines detectors
detector = vehicleDetectorFasterRCNN();
pause % press a key to continue
while v.hasFrame
    vidFrame = readFrame(v);

    
    % Applies detector on img
    [bboxes,scores] = detect(detector,vidFrame);
    
    % Conditions and defining good and bad bboxes
    mask = scores > thresh;
    gbboxes = bboxes;
    gscores = scores;
    bbboxes = bboxes;
    bscores = scores;
    gbboxes(~mask, :) = [];
    gscores(~mask, :) = [];
    bbboxes(mask, :) = [];
    bscores(mask, :) = [];

    % Inserts an annotation on vehicles detected
    if ~isempty(gscores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',gbboxes,gscores,'Color', 'Green');
    end

    if ~isempty(bscores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',bbboxes,bscores,'Color', 'Red');
    end

    img.CData = vidFrame ;
    str = sprintf('Current Time = %.3f sec', v.CurrentTime);
    tt.String = str;
    pause(1/v.FrameRate)
end

%% Exercise 6
clear, clc, close all
% Load video
vFile = 'video.mp4';
v = VideoReader(vFile);
v.CurrentTime = 0;
vidFrame = readFrame(v);
[h, w, c] = size(vidFrame);
img = imshow(vidFrame);
hold on
tt=title(sprintf('Current Time = %.3f sec', v.CurrentTime));
thresh = 20;
% Defines detectors
detector = vehicleDetectorACF();
pause % press a key to continue
while v.hasFrame
    vidFrame = readFrame(v);

    
    % Applies detector on img
    [bboxes,scores] = detect(detector,vidFrame);
    
    % Conditions and defining good and bad bboxes
    mask = scores > thresh;
    gbboxes = bboxes;
    gscores = scores;
    bbboxes = bboxes;
    bscores = scores;
    gbboxes(~mask, :) = [];
    gscores(~mask, :) = [];
    gcent = gbboxes(:, 1:2) + gbboxes(:, 3:4)/2;
    bbboxes(mask, :) = [];
    bscores(mask, :) = [];
    bcent = bbboxes(:, 1:2) + bbboxes(:, 3:4)/2;

    % Inserts an annotation on vehicles detected
    if ~isempty(gscores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',gbboxes,gscores,'Color', 'Green');
        plot(gcent(:,1), gcent(:,2), '.g')
    end

    if ~isempty(bscores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',bbboxes,bscores,'Color', 'Red');
        plot(bcent(:,1), bcent(:,2), '.r')
    end
    
    img.CData = vidFrame ;
    str = sprintf('Current Time = %.3f sec', v.CurrentTime);
    tt.String = str;
    pause(1/v.FrameRate)
end

%% Exercise 7
clear, clc, close all
% Load video
vFile = 'video.mp4';
v = VideoReader(vFile);
v.CurrentTime = 0;
vidFrame = readFrame(v);
[h, w, c] = size(vidFrame);
img = imshow(vidFrame);
hold on
tt=title(sprintf('Current Time = %.3f sec', v.CurrentTime));

% Defines detectors and variables
detector = vehicleDetectorACF();
thresh = 20;
gcents =  [];

% Define line
l = line([0, w], [h/2 h/2], 'Color', 'yellow', 'LineWidth', 2);

% Define ransac parameters
sampleSize = 2; % number of points to sample per trial
maxDistance = 4; % max allowable distance for inliers
% fit function using polyfit with order 1 polynomial (straight line)
fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1);
% distance evaluation function (total euclidian distance)
evalLineFcn = @(model, points) sum((points(:, 2) - polyval(model, ...
points(:,1))).^2,2);

pause % press a key to continue
while v.hasFrame
    vidFrame = readFrame(v);

    
    % Applies detector on img
    [bboxes,scores] = detect(detector,vidFrame);
    
    % Conditions and defining good and bad bboxes
    mask = scores > thresh;
    gbboxes = bboxes;
    gscores = scores;
    bbboxes = bboxes;
    bscores = scores;
    gbboxes(~mask, :) = [];
    gscores(~mask, :) = [];
    gcent = gbboxes(:, 1:2) + gbboxes(:, 3:4)/2;
    bbboxes(mask, :) = [];
    bscores(mask, :) = [];
    bcent = bbboxes(:, 1:2) + bbboxes(:, 3:4)/2;

    % Inserts an annotation on vehicles detected
    if ~isempty(gscores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',gbboxes,gscores,'Color', 'Green');
        plot(gcent(:,1), gcent(:,2), '.g')
        gcents = [gcents; gcent];
    end

    if ~isempty(bscores)
        vidFrame = insertObjectAnnotation(vidFrame,'rectangle',bbboxes,bscores,'Color', 'Red');
        plot(bcent(:,1), bcent(:,2), '.r')
    end
    
    if length(gcents(:,1)) >= 5
        [model, inlier] = ransac(gcents, fitLineFcn, evalLineFcn, sampleSize, maxDistance);
        Y = polyval(model, [0, w]);
        set(l, 'YData', Y)
    end

    img.CData = vidFrame ;
    str = sprintf('Current Time = %.3f sec', v.CurrentTime);
    tt.String = str;
    pause(1/v.FrameRate)
end