%% Exercise 1
clear, clc, close all
load lidarScans.mat
s = lidarScans(250);
sCart = s.Cartesian;
subplot(1,2,1)
plot(sCart(:,1), sCart(:,2),'.k');
axis equal
grid on
xlabel('X')
ylabel('Y')
% [x,y] = pol2cart(s.Angles,s.Ranges);
% hold on
% plot(x, y,'or');
axis([-5 5 -8 3])
subplot(1,2,2)
h = s.plot;
h.MarkerSize=3;
h.MarkerEdgeColor = 'k';
view(0, 90);
axis([-5 5 -8 3])

%% Exercise 2
clear, clc, close all
load lidarScans.mat
robot=[  0    0.5  0;
        -0.2  0    0.2];
first = lidarScans(1);
h = first.plot;
axis equal; grid on;
totSavedScans = [];
for n=1:numel(lidarScans)
    hold off
    lidarData = lidarScans(n);
    minRange = 0.15;
    maxRange = 8;
    lidarData = removeInvalidData(lidarData,'RangeLimits',[minRange maxRange]);
    lidarData.plot
    lidarCart = lidarData.Cartesian;
    totSavedScans=[totSavedScans, size(lidarCart,1)];
    hold on
    fill([0; lidarCart(:,1)], [0; lidarCart(:,2)], 'y')
    fill(robot(1,:), robot(2,:),'r');
    axis([-8 8 -8 8])
    pause(0.01)
end
minPoints=min(totSavedScans);
find(totSavedScans==minPoints)

%% Exercise 3
clear, clc, close all
load lidarScans.mat
el = numel(lidarScans);
robot=[  0    0.5  0;
        -0.2  0    0.2];
first = lidarScans(1);
f = figure;
f.Position = [0 0 1366 768];
h = first.plot;
h.Color = 'k';
axis equal; grid on; axis([-8 8 -8 8]);
titlestr = sprintf ('Scan 1 of %d', el);
title(titlestr)
step = 5;
for n = step+1:el
    hold off
    lidarData = lidarScans(n);
    current = lidarData.plot;
    current.Color = 'k';
    hold on; 
    previousData = lidarScans(n-step);
    previous = previousData.plot;
    previous.Color = 'r';
    [pose, stats] = matchScans(lidarData,previousData);
    estScan = transformScan(previousData,pose);
    est = estScan.plot;
    est.Color = 'b';
    fill(robot(1,:), robot(2,:),'c');
    axis([-8 8 -8 8]);
    leg = legend('Current','Previous','Estimated','Robot');
    rect = [0.1, 0.1, 0.1, 0.1];
    leg.Position = rect;
    str=sprintf('Scan=%d, Score= %d, Pose=[%.0f mm, %.0f mm, %.2fº]', n, round(stats.Score), pose .* [1000 1000 180/pi]); title(str);
    pause(0.01)

% TODO Verify if estScan is correct
end

%% Exercise 4
clear, clc, close all
load lidarScans.mat
numScans = numel(lidarScans);
robot=[  0    0.5  0;
        -0.2  0    0.2];
initialPose = [0 0 0]; %initial estimate of pose
poseList = zeros(numScans,3); %empty list to calculate poses
poseList(1,:) = initialPose;
scoreList = zeros(numScans);


step = 1;
for n = step+1:numScans
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);
    [pose, stats] = matchScans(currScan,prevScan);
    poseList(n,:) = pose;
end

figure(5)
hold on
grid on
fill(robot(1,:), robot(2,:),'c')

T = eye(3);
for n = 2:size(poseList,1)
    T = T * transform2D(poseList(n,:)); %T is the accumulated total

    pose = [T(1,3), T(2,3), atan2(T(2,1),T(1,1)) ]; %pose is the current iteration real pose

    robotf = T * [robot; 1 1 1];

    fill(robotf(1,:), robotf(2,:),'c');
end

%% Exercise 5 
clear, clc, close all
load lidarScans.mat
numScans = numel(lidarScans);
robot=[  0    0.5  0;
        -0.2  0    0.2];
initialPose = [0 0 0]; %initial estimate of pose
poseList = zeros(numScans,3); %empty list to calculate poses
poseList(1,:) = initialPose;
scoreList = zeros(numScans);

map = occupancyMap(15,15,20);
map.GridLocationInWorld = [-7.5 -7.5];


T = eye(3);

step = 1;

figure(5)
hold on
grid on

for n = step+1:numScans
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);
    [pose, stats] = matchScans(currScan,prevScan);
    T = T * transform2D(pose); %T is the accumulated total
    pose = [T(1,3), T(2,3), atan2(T(2,1),T(1,1)) ]; %pose is the current iteration real pose
    poseList(n,:) = pose;
    insertRay(map, pose, currScan, 10);
    robotf = T * [robot; 1 1 1];
    fill(robotf(1,:), robotf(2,:),'c');
end

show(map)


