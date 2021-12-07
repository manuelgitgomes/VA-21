%% TP1
% Add to path
addpath('TP1/')

% Clearing
clear, clc, close all

% Starting variables
radarObj = [];
cameraObj = [];
carObj = [];
barObj = [];
cycObj = [];
pedObj = [];
carNum = 0;
cycNum = 0;
pedNum = 0;
posDet = zeros(2, 2);
carDist = 5;
cycDist = 3;
pedDist = 1.75;
count = false;

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

% Creating plots
subplot(2, 3, 1)
plot(PP(:,1), PP(:,2), 'k', 'DisplayName', 'Vehicle Trajectory')
view(-90, 90)
title('Vehicle Trajectory')
axis square
xlabel('X (m)')
ylabel('Y (m)')

subplot(2, 3, 2)
plot(PP(:,1), PP(:,2), 'k', 'DisplayName', 'Vehicle Trajectory')
hold on
view(-90, 90)
title('Object Detections with Radar')
axis square
xlabel('X (m)')
ylabel('Y (m)')
legend

subplot(2, 3, 3)
plot(PP(:,1), PP(:,2), 'k', 'DisplayName', 'Vehicle Trajectory')
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

    % If there are detections, retrieve ego car pose and calculate the
    % transform matrix
    if numel(objs) ~= 0
        posCar=PP(n,:);
        orCar = fliplr(allData(n).INSMeasurements{1,1}.Orientation) * pi/180;
        TCtrans = trvec2tform(posCar);
        TCrot = eul2tform(orCar);
        TCar = TCtrans * TCrot;

        % Retrieve object pose, calculate its transform matrix and
        % calculate object absolute coords
        for i=1:numel(objs)
            typeObj = objs{i}.SensorIndex;
            classObj = objs{i}.ObjectClassID;
            posObj = objs{i}.Measurement(1:3)';
            orObj = (objs{i}.Measurement(4:6))' * pi/180;
            TOtrans = trvec2tform(posObj);
            TOrot = eul2tform(orObj);
            TObj = TOtrans * TOrot;
            Pmundo = TCar * TObj * [0, 0, 0, 1]';

            % Separating detection by sensor and by object detected
            if typeObj == 3
                radarObj(end+1,:) = Pmundo;
            elseif typeObj == 4 || typeObj == 5 || typeObj == 6 || typeObj == 7|| typeObj == 8
                cameraObj(end+1,:) = Pmundo;
                if classObj == 1 || classObj == 2
                    carObj(end+1,:) = Pmundo;
                elseif classObj == 3
                    cycObj(end+1,:) = Pmundo;
                elseif classObj == 4
                    pedObj(end+1,:) = Pmundo;
                elseif classObj == 5 || classObj == 6
                    barObj(end+1,:) = Pmundo;
                end
        
            end
        end
    end
end


% Plot detections
subplot(2, 3, 2)
plot(radarObj(:,1), radarObj(:,2), 'or', 'DisplayName', 'Radar Detections')

subplot(2, 3, 3)
plot(carObj(:,1), carObj(:,2), 'or', 'DisplayName', 'Vehicle Detections')
plot(cycObj(:,1), cycObj(:,2), 'og', 'DisplayName', 'Bicycle Detections')
plot(pedObj(:,1), pedObj(:,2), 'ob', 'DisplayName', 'Pedestrian Detections')

% Order matrixes
carObj = sortrows(carObj(:, [1, 2]));
cycObj = sortrows(cycObj(:, [1, 2]));
pedObj = sortrows(pedObj(:, [1, 2]));


% Calculating the number of objects
for i = 1:length(carObj(:,1))
    posDet(1, :) = carObj(i,:);
    for ii = i+1:length(carObj(:,1))
        posDet(2, :) = carObj(ii,:);
        distance = pdist(posDet, 'euclidean');
        if distance < carDist
            if ~count
                carNum = carNum + 1;
                count = true;
            end
            break
        elseif ii == length(carObj(2:end,1))
            count = false;
        end
    end
end

for i = 1:length(cycObj(:,1))
    posDet(1, :) = cycObj(i,:);
    for ii = i+1:length(cycObj(:,1))
        posDet(2, :) = cycObj(ii,:);
        distance = pdist(posDet, 'euclidean');
        if distance < cycDist
            if ~count
                cycNum = cycNum + 1;
                count = true;
            end
            break
        elseif ii == length(cycObj(2:end,1))
            count = false;
        end
    end
end

for i = 1:length(pedObj(:,1))
    posDet(1, :) = pedObj(i,:);
    for ii = i+1:length(pedObj(:,1))
        posDet(2, :) = pedObj(ii,:);
        distance = pdist(posDet, 'euclidean');
        if distance < pedDist
            if ~count
                pedNum = pedNum + 1;
                count = true;
            end
            break
        elseif ii == length(pedObj(2:end,1))
            count = false;
        end
    end
end
