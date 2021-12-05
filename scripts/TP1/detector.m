%% TP1
% Add to path
addpath('TP1/')

% Clearing
clear, clc, close all

% Starting variables
radarObj = [];
cameraObj = [];

% Load data
[allData, scenario, sensors] = scene1();

% Erasing Actor Pose
for n=1:numel(allData)
    allData(n).ActorPoses(2:end)=[];
end

% Defining time
t = [allData.Time];

% Plotting ego position with Actor Poses
% PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', allData, 'UniformOutput', false))';

% Plotting ego position with IMS Measurements
PP = cell2mat(arrayfun(@(S) S.INSMeasurements{1}.Position', allData, 'UniformOutput', false))';

% Viewing position
subplot(2, 3, 1)
plot(PP(:,1), PP(:,2), 'r', 'DisplayName', 'Vehicle Trajectory')
view(-90, 90)
title('Vehicle Trajectory')
axis square
xlabel('X (m)')
ylabel('Y (m)')

subplot(2, 3, 2)
plot(PP(:,1), PP(:,2), 'r', 'DisplayName', 'Vehicle Trajectory')
hold on
view(-90, 90)
title('Object Detections with Radar')
axis square
xlabel('X (m)')
ylabel('Y (m)')
legend

subplot(2, 3, 3)
plot(PP(:,1), PP(:,2), 'r', 'DisplayName', 'Vehicle Trajectory')
hold on
view(-90, 90)
title('Object Detections with Camera')
axis square
xlabel('X (m)')
ylabel('Y (m)')
legend


% Detecting objects with radar
for n=1:numel(allData)
    objs=allData(n).ObjectDetections;
    if numel(objs) ~= 0
        posCar=PP(n,:);
        orCar = fliplr(allData(n).INSMeasurements{1,1}.Orientation) * pi/180;
        TCtrans = trvec2tform(posCar);
        TCrot = eul2tform(orCar);
        TCar = TCtrans * TCrot;
        for i=1:numel(objs)
            typeObj = objs{i}.SensorIndex;
            posObj = objs{i}.Measurement(1:3)';
            orObj = (objs{i}.Measurement(4:6))' * pi/180;
            TOtrans = trvec2tform(posObj);
            TOrot = eul2tform(orObj);
            TObj = TOtrans * TOrot;
            Pmundo = TCar * TObj * [0, 0, 0, 1]';
            if typeObj == 3
                radarObj(end+1,:) = Pmundo;
            elseif typeObj == 4 || typeObj == 5 || typeObj == 6 || typeObj == 7|| typeObj == 8
                cameraObj(end+1,:) = Pmundo;
            end
        end
    end
end

subplot(2, 3, 2)
plot(radarObj(:,1), radarObj(:,2), 'og', 'DisplayName', 'Radar Detections')

subplot(2, 3, 3)
plot(cameraObj(:,1), cameraObj(:,2), 'ob', 'DisplayName', 'Camera Detections')

