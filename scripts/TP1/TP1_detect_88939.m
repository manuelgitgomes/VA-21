%% TP1
% Add to path
addpath('TP1/')

% Clearing
clear, clc, close all

% Load data
[allData, scenario, sensors] = TP1_DSD_88939();

% Starting variables
radarObj = [];
cameraObj = [];
carObj = [];
truObj = [];
barObj = [];
cycObj = [];
pedObj = [];
carNum = 0;
truNum = 0;
cycNum = 0;
pedNum = 0;
barNum = 0;
carmovNum = 0;
trumovNum = 0;
carCoords = {};
carStopCoords = {};
truCoords = {};
truStopCoords = {};
cycCoords = {};
pedCoords = {};
barCoords = {};
carCent = [];
carStopCent = [];
truCent = [];
truStopCent = [];
posDet = zeros(2, 2);
objDist = 4;
truDist = 6;
count = false;
object = false;
PPAPDist = 0;
IMSDist = 0;
interval = 1;
distDet = zeros(2, 2);
imsDet = zeros(2, 2);
counter = 0;
unused = [];
carMov = 6;
truMov = 12; % TODO verify this value
mindistCar = 0;
mindistIdx = inf;
mindistFirstBar = 0;
mindistFirstIdx = inf;
maxdistLastBar = 0;
maxdistLastIdx = 0;
laneX = zeros(1, length(allData));
laneL = [];
laneR = [];


% Erasing Actor Pose
for n=1:numel(allData)
    allData(n).ActorPoses(2:end)=[];
end

% Defining time
t = [allData.Time];

% Plotting ego position with Actor Poses
PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', allData, 'UniformOutput', false))';

% Using Actor Poses
for i = 1:ceil(length(PP)/interval)-1
    imsDet(1, :) = PP(interval*i);
    if i == ceil(length(PP)/interval)-1
        imsDet(2, :) = PP(end);
    else
        imsDet(2, :) = PP(interval*(i+1));
    end
    IMSDist = IMSDist + pdist(imsDet);
end

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
title('Detections with Radar')
axis square
xlabel('X (m)')
ylabel('Y (m)')
legend

subplot(2, 3, 3)
plot(PP(:,1), PP(:,2), 'k', 'DisplayName', 'Vehicle Trajectory')
hold on
view(-90, 90)
title('Detections with Camera')
axis square
xlabel('X (m)')
ylabel('Y (m)')
legend

subplot(2, 3, 4)
plot(PP(:,1), PP(:,2), 'k', 'DisplayName', 'Vehicle Trajectory')
hold on
view(-90, 90)
title('Object Detections')
axis square
xlabel('X (m)')
ylabel('Y (m)')
legend

subplot(2, 3, 5)
plot(PP(:,1), PP(:,2), 'k', 'DisplayName', 'Vehicle Trajectory')
hold on
view(-90, 90)
title('Vehicles Centroids')
axis square
xlabel('X (m)')
ylabel('Y (m)')
legend


subplot(2, 3, 6)
plot(PP(:,1), PP(:,2), 'k', 'DisplayName', 'Vehicle Trajectory')
hold on
view(-90, 90)
title('Distances')
axis square
xlabel('X (m)')
ylabel('Y (m)')
legend




% Detecting objects with radar
for n=1:numel(allData)
    objs = allData(n).ObjectDetections;
    lanes = allData(n).LaneDetections;

    % If there are detections, retrieve ego car pose and calculate the
    % transform matrix
    if numel(objs) ~= 0 || numel(lanes) ~= 0
        posCar=PP(n,:);
        orCar = fliplr(allData(n).INSMeasurements{1,1}.Orientation) * pi/180;
        TCtrans = trvec2tform(posCar);
        TCrot = eul2tform(orCar);
        TCar = TCtrans * TCrot;

        % Defining lane detections
        laneDetection = allData(n).LaneDetections(1).LaneBoundaries;
        leftLane = laneDetection(1,1);
        rightLane = laneDetection(1,2);

        leftY = computeBoundaryModel(leftLane(1), laneX);
        rigthY = computeBoundaryModel(rightLane(1), laneX);

        leftPos = [leftLane(1).LateralOffset leftY(n) 0];
        rightPos = [rightLane(1).LateralOffset rigthY(n) 0];
        TLtrans = trvec2tform(leftPos);
        TRtrans = trvec2tform(rightPos);
        Pleft = TCar * TLtrans * [0, 0, 0, 1]';
        Pright = TCar * TRtrans * [0, 0, 0, 1]';
        laneL(end+1, :) = Pleft(1:2, 1);
        laneR(end+1, :) = Pright(1:2, 1);

            
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
            elseif typeObj == 4 || typeObj == 5 || typeObj == 6 || typeObj == 7|| typeObj == 8 || typeObj == 9|| typeObj == 10
                cameraObj(end+1,:) = Pmundo;
                if classObj == 1 
                    carObj(end+1,:) = Pmundo;
                elseif classObj == 2
                    truObj(end+1,:) = Pmundo;
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
% Plot lanes
laneL = rmmissing(laneL);
laneR = rmmissing(laneR);
plot(laneL(:,1), laneL(:,2), 'r', 'DisplayName', 'Left Lane')
plot(laneR(:,1), laneR(:,2), 'g', 'DisplayName', 'Right Lane')

% Defining barriers
barObj = radarObj;
barMask = zeros(size(barObj));

for i = 1:length(radarObj)
    for ii = 1:length(cameraObj)
        distance = pdist([radarObj(i,1:2); cameraObj(ii,1:2)]);
        if distance < objDist
            barMask(i, :) = [1, 1, 1, 1];
            break
        end
    end
end
barObj(logical(barMask(:,1)),:) = [];

% Plot detections
subplot(2, 3, 2)
plot(radarObj(:,1), radarObj(:,2), 'or', 'DisplayName', 'Radar Detections')

subplot(2, 3, 3)
plot(cameraObj(:,1), cameraObj(:,2), 'or', 'DisplayName', 'Camera Detections')

subplot(2, 3, 4)
plot(carObj(:,1), carObj(:,2), 'or', 'DisplayName', 'Car Detections')
plot(truObj(:,1), truObj(:,2), 'om', 'DisplayName', 'Truck Detections')
plot(cycObj(:,1), cycObj(:,2), 'og', 'DisplayName', 'Bicycle Detections')
plot(pedObj(:,1), pedObj(:,2), 'ob', 'DisplayName', 'Pedestrian Detections')

% Order matrixes
sortcarObj = carObj(:, [1, 2]);
sorttruObj = truObj(:, [1, 2]);
sortcycObj = cycObj(:, [1, 2]);
sortpedObj = pedObj(:, [1, 2]);
sortbarObj = barObj(:, [1, 2]);


% Calculating the number of objects

% For every car detections, detect if there are points on its vicinity
for i = 1:length(sortcarObj(:,1))
    curr = sortcarObj(i,:);
    dist = sqrt((sortcarObj(i+1:end,1) - curr(1)).^2 + (sortcarObj(i+1:end,2) - curr(2)).^2);
    mask = dist < objDist;
    num = sum(mask);
    str = strcat('Found ', int2str(num), ' cars nearby.');
    disp(str)
    if num > 0
        if ~count
            carNum = carNum + 1;
            count = true;
            carCoords{end + 1} = [];
        end
        % Sort the closest point to be the next one chosen
        carCoords{end}(end+1, :) = curr;
        idxs = find(mask) + i;
        minIdx = find(dist == min(dist)) + i;
        v = sortcarObj(minIdx, :);
       
        idxs = idxs(~(idxs==minIdx));
       
        % If more then one point is detected, record its coordinates to
        % work on the next iteration
        for ii = 1:length(idxs)
            un = sortcarObj(idxs(ii),:);
            if isempty(unused)
                unused(end+1, :) = un;
            else
                [~,index] = ismember(unused,un,'rows');
                if sum(index) == 0
                    unused(end+1, :) = un;
                end
            end
        end
        sortcarObj(minIdx, :) = sortcarObj(i+1, :);
        sortcarObj(i+1, :) = v;
        if ~isempty(unused)
            [~,index] = ismember(unused, v ,'rows');
            unused = unused(~index, :);
            [~,index] = ismember(unused, curr,'rows');
            unused = unused(~index, :);
        end
    % Working on the previously detected points
    elseif ~isempty(unused)
        [idx, ~] = find(sortcarObj==unused(1,:));
        unused = unused(2:end, :);
        curr = sortcarObj(idx(1), :);
        sortcarObj(idx(1), :) = sortcarObj(i, :);
        sortcarObj(i, :) = curr;
        dist = sqrt((sortcarObj(i+1:end,1) - curr(1)).^2 + (sortcarObj(i+1:end,2) - curr(2)).^2);
        mask = dist < objDist;
        num = sum(mask);
        str = strcat('Found ', int2str(num), ' cars nearby, after backtracking.');
        disp(str)
        if num > 0
            carCoords{end}(end+1, :) = curr;
            idxs = find(mask) + i;
            minIdx = find(dist == min(dist)) + i;
            v = sortcarObj(minIdx, :);
           
            idxs = idxs(~(idxs==minIdx));
           
            for ii = 1:length(idxs)
                un = sortcarObj(idxs(ii),:);
                if isempty(unused)
                    unused(end+1, :) = un;
                else
                    [~,index] = ismember(unused,un,'rows');
                    if sum(index) == 0
                        unused(end+1, :) = un;
                    end
                end
            end
            sortcarObj(minIdx, :) = sortcarObj(i+1, :);
            sortcarObj(i+1, :) = v;
            if ~isempty(unused)
                [~,index] = ismember(unused, v,'rows');
                unused = unused(~index, :);
                [~,index] = ismember(unused, curr,'rows');
                unused = unused(~index, :);
            end
        elseif isempty(unused)
            disp('End of Object')
            count = false;
        end
    else
        disp('Solo Object')
        count = false;
    end
    % Eliminating already calculated points
     sortcarObj(i,:) = NaN;
end

% For every car detections, detect if there are points on its vicinity
for i = 1:length(sorttruObj(:,1))
    curr = sorttruObj(i,:);
    dist = sqrt((sorttruObj(i+1:end,1) - curr(1)).^2 + (sorttruObj(i+1:end,2) - curr(2)).^2);
    mask = dist < truDist;
    num = sum(mask);
    str = strcat('Found ', int2str(num), ' trucks nearby.');
    disp(str)
    if num > 0
        if ~count
            truNum = truNum + 1;
            count = true;
            truCoords{end + 1} = [];
        end
        % Sort the closest point to be the next one chosen
        truCoords{end}(end+1, :) = curr;
        idxs = find(mask) + i;
        minIdx = find(dist == min(dist)) + i;
        v = sorttruObj(minIdx, :);
       
        idxs = idxs(~(idxs==minIdx));
       
        % If more then one point is detected, record its coordinates to
        % work on the next iteration
        for ii = 1:length(idxs)
            un = sorttruObj(idxs(ii),:);
            if isempty(unused)
                unused(end+1, :) = un;
            else
                [~,index] = ismember(unused,un,'rows');
                if sum(index) == 0
                    unused(end+1, :) = un;
                end
            end
        end
        sorttruObj(minIdx, :) = sorttruObj(i+1, :);
        sorttruObj(i+1, :) = v;
        if ~isempty(unused)
            [~,index] = ismember(unused, v ,'rows');
            unused = unused(~index, :);
            [~,index] = ismember(unused, curr,'rows');
            unused = unused(~index, :);
        end
    % Working on the previously detected points
    elseif ~isempty(unused)
        [idx, ~] = find(sorttruObj==unused(1,:));
        unused = unused(2:end, :);
        curr = sorttruObj(idx(1), :);
        sorttruObj(idx(1), :) = sorttruObj(i, :);
        sorttruObj(i, :) = curr;
        dist = sqrt((sorttruObj(i+1:end,1) - curr(1)).^2 + (sorttruObj(i+1:end,2) - curr(2)).^2);
        mask = dist < truDist;
        num = sum(mask);
        str = strcat('Found ', int2str(num), ' trucks nearby, after backtracking.');
        disp(str)
        if num > 0
            truCoords{end}(end+1, :) = curr;
            idxs = find(mask) + i;
            minIdx = find(dist == min(dist)) + i;
            v = sorttruObj(minIdx, :);
           
            idxs = idxs(~(idxs==minIdx));
           
            for ii = 1:length(idxs)
                un = sorttruObj(idxs(ii),:);
                if isempty(unused)
                    unused(end+1, :) = un;
                else
                    [~,index] = ismember(unused,un,'rows');
                    if sum(index) == 0
                        unused(end+1, :) = un;
                    end
                end
            end
            sorttruObj(minIdx, :) = sorttruObj(i+1, :);
            sorttruObj(i+1, :) = v;
            if ~isempty(unused)
                [~,index] = ismember(unused, v,'rows');
                unused = unused(~index, :);
                [~,index] = ismember(unused, curr,'rows');
                unused = unused(~index, :);
            end
        elseif isempty(unused)
            disp('End of Object')
            count = false;
        end
    else
        disp('Solo Object')
        count = false;
    end
    % Eliminating already calculated points
     sorttruObj(i,:) = NaN;
end

% For every cyc detections, detect if there are points on its vicinity
for i = 1:length(sortcycObj(:,1))
    curr = sortcycObj(i,:);
    dist = sqrt((sortcycObj(i+1:end,1) - curr(1)).^2 + (sortcycObj(i+1:end,2) - curr(2)).^2);
    mask = dist < objDist;
    num = sum(mask);
    str = strcat('Found ', int2str(num), ' cycs nearby.');
    disp(str)
    if num > 0
        if ~count
            cycNum = cycNum + 1;
            count = true;
            cycCoords{end + 1} = [];
        end
        % Sort the closest point to be the next one chosen
        cycCoords{end}(end+1, :) = curr;
        idxs = find(mask) + i;
        minIdx = find(dist == min(dist)) + i;
        v = sortcycObj(minIdx, :);
       
        idxs = idxs(~(idxs==minIdx));
       
        % If more then one point is detected, record its coordinates to
        % work on the next iteration
        for ii = 1:length(idxs)
            un = sortcycObj(idxs(ii),:);
            if isempty(unused)
                unused(end+1, :) = un;
            else
                [~,index] = ismember(unused,un,'rows');
                if sum(index) == 0
                    unused(end+1, :) = un;
                end
            end
        end
        sortcycObj(minIdx, :) = sortcycObj(i+1, :);
        sortcycObj(i+1, :) = v;
        if ~isempty(unused)
            [~,index] = ismember(unused, v ,'rows');
            unused = unused(~index, :);
            [~,index] = ismember(unused, curr,'rows');
            unused = unused(~index, :);
        end
    % Working on the previously detected points
    elseif ~isempty(unused)
        [idx, ~] = find(sortcycObj==unused(1,:));
        unused = unused(2:end, :);
        curr = sortcycObj(idx(1), :);
        sortcycObj(idx(1), :) = sortcycObj(i, :);
        sortcycObj(i, :) = curr;
        dist = sqrt((sortcycObj(i+1:end,1) - curr(1)).^2 + (sortcycObj(i+1:end,2) - curr(2)).^2);
        mask = dist < objDist;
        num = sum(mask);
        str = strcat('Found ', int2str(num), ' cycs nearby, after backtracking.');
        disp(str)
        if num > 0
            cycCoords{end}(end+1, :) = curr;
            idxs = find(mask) + i;
            minIdx = find(dist == min(dist)) + i;
            v = sortcycObj(minIdx, :);
           
            idxs = idxs(~(idxs==minIdx));
           
            for ii = 1:length(idxs)
                un = sortcycObj(idxs(ii),:);
                if isempty(unused)
                    unused(end+1, :) = un;
                else
                    [~,index] = ismember(unused,un,'rows');
                    if sum(index) == 0
                        unused(end+1, :) = un;
                    end
                end
            end
            sortcycObj(minIdx, :) = sortcycObj(i+1, :);
            sortcycObj(i+1, :) = v;
            if ~isempty(unused)
                [~,index] = ismember(unused, v,'rows');
                unused = unused(~index, :);
                [~,index] = ismember(unused, curr,'rows');
                unused = unused(~index, :);
            end
        elseif isempty(unused)
            disp('End of Object')
            count = false;
        end
    else
        disp('Solo Object')
        count = false;
    end
    % Eliminating already calculated points
     sortcycObj(i,:) = NaN;
end

% For every pedestrian detections, detect if there are points on its vicinity
for i = 1:length(sortpedObj(:,1))
    curr = sortpedObj(i,:);
    dist = sqrt((sortpedObj(i+1:end,1) - curr(1)).^2 + (sortpedObj(i+1:end,2) - curr(2)).^2);
    mask = dist < objDist;
    num = sum(mask);
    str = strcat('Found ', int2str(num), ' peds nearby.');
    disp(str)
    if num > 0
        if ~count
            pedNum = pedNum + 1;
            count = true;
            pedCoords{end + 1} = [];
        end
        % Sort the closest point to be the next one chosen
        pedCoords{end}(end+1, :) = curr;
        idxs = find(mask) + i;
        minIdx = find(dist == min(dist)) + i;
        v = sortpedObj(minIdx, :);
       
        idxs = idxs(~(idxs==minIdx));
       
        % If more then one point is detected, record its coordinates to
        % work on the next iteration
        for ii = 1:length(idxs)
            un = sortpedObj(idxs(ii),:);
            if isempty(unused)
                unused(end+1, :) = un;
            else
                [~,index] = ismember(unused,un,'rows');
                if sum(index) == 0
                    unused(end+1, :) = un;
                end
            end
        end
        sortpedObj(minIdx, :) = sortpedObj(i+1, :);
        sortpedObj(i+1, :) = v;
        if ~isempty(unused)
            [~,index] = ismember(unused, v ,'rows');
            unused = unused(~index, :);
            [~,index] = ismember(unused, curr,'rows');
            unused = unused(~index, :);
        end
    % Working on the previously detected points
    elseif ~isempty(unused)
        [idx, ~] = find(sortpedObj==unused(1,:));
        unused = unused(2:end, :);
        curr = sortpedObj(idx(1), :);
        sortpedObj(idx(1), :) = sortpedObj(i, :);
        sortpedObj(i, :) = curr;
        dist = sqrt((sortpedObj(i+1:end,1) - curr(1)).^2 + (sortpedObj(i+1:end,2) - curr(2)).^2);
        mask = dist < objDist;
        num = sum(mask);
        str = strcat('Found ', int2str(num), ' peds nearby, after backtracking.');
        disp(str)
        if num > 0
            pedCoords{end}(end+1, :) = curr;
            idxs = find(mask) + i;
            minIdx = find(dist == min(dist)) + i;
            v = sortpedObj(minIdx, :);
           
            idxs = idxs(~(idxs==minIdx));
           
            for ii = 1:length(idxs)
                un = sortpedObj(idxs(ii),:);
                if isempty(unused)
                    unused(end+1, :) = un;
                else
                    [~,index] = ismember(unused,un,'rows');
                    if sum(index) == 0
                        unused(end+1, :) = un;
                    end
                end
            end
            sortpedObj(minIdx, :) = sortpedObj(i+1, :);
            sortpedObj(i+1, :) = v;
            if ~isempty(unused)
                [~,index] = ismember(unused, v,'rows');
                unused = unused(~index, :);
                [~,index] = ismember(unused, curr,'rows');
                unused = unused(~index, :);
            end
        elseif isempty(unused)
            disp('End of Object')
            count = false;
        end
    else
        disp('Solo Object')
        count = false;
    end
    % Eliminating already calculated points
     sortpedObj(i,:) = NaN;
end

% For every bar detections, detect if there are points on its vicinity
for i = 1:length(sortbarObj(:,1))
    curr = sortbarObj(i,:);
    dist = sqrt((sortbarObj(i+1:end,1) - curr(1)).^2 + (sortbarObj(i+1:end,2) - curr(2)).^2);
    mask = dist < objDist;
    num = sum(mask);
    str = strcat('Found ', int2str(num), ' bars nearby.');
    disp(str)
    if num > 0
        if ~count
            barNum = barNum + 1;
            count = true;
            barCoords{end + 1} = [];
        end
        % Sort the closest point to be the next one chosen
        barCoords{end}(end+1, :) = curr;
        idxs = find(mask) + i;
        minIdx = find(dist == min(dist)) + i;
        v = sortbarObj(minIdx, :);
       
        idxs = idxs(~(idxs==minIdx));
       
        % If more then one point is detected, record its coordinates to
        % work on the next iteration
        for ii = 1:length(idxs)
            un = sortbarObj(idxs(ii),:);
            if isempty(unused)
                unused(end+1, :) = un;
            else
                [~,index] = ismember(unused,un,'rows');
                if sum(index) == 0
                    unused(end+1, :) = un;
                end
            end
        end
        sortbarObj(minIdx, :) = sortbarObj(i+1, :);
        sortbarObj(i+1, :) = v;
        if ~isempty(unused)
            [~,index] = ismember(unused, v ,'rows');
            unused = unused(~index, :);
            [~,index] = ismember(unused, curr,'rows');
            unused = unused(~index, :);
        end
    % Working on the previously detected points
    elseif ~isempty(unused)
        [idx, ~] = find(sortbarObj==unused(1,:));
        unused = unused(2:end, :);
        curr = sortbarObj(idx(1), :);
        sortbarObj(idx(1), :) = sortbarObj(i, :);
        sortbarObj(i, :) = curr;
        dist = sqrt((sortbarObj(i+1:end,1) - curr(1)).^2 + (sortbarObj(i+1:end,2) - curr(2)).^2);
        mask = dist < objDist;
        num = sum(mask);
        str = strcat('Found ', int2str(num), ' peds nearby, after backtracking.');
        disp(str)
        if num > 0
            barCoords{end}(end+1, :) = curr;
            idxs = find(mask) + i;
            minIdx = find(dist == min(dist)) + i;
            v = sortbarObj(minIdx, :);
           
            idxs = idxs(~(idxs==minIdx));
           
            for ii = 1:length(idxs)
                un = sortbarObj(idxs(ii),:);
                if isempty(unused)
                    unused(end+1, :) = un;
                else
                    [~,index] = ismember(unused,un,'rows');
                    if sum(index) == 0
                        unused(end+1, :) = un;
                    end
                end
            end
            sortbarObj(minIdx, :) = sortbarObj(i+1, :);
            sortbarObj(i+1, :) = v;
            if ~isempty(unused)
                [~,index] = ismember(unused, v,'rows');
                unused = unused(~index, :);
                [~,index] = ismember(unused, curr,'rows');
                unused = unused(~index, :);
            end
        elseif isempty(unused)
            disp('End of Object')
            count = false;
        end
    else
        disp('Solo Object')
        count = false;
    end
    % Eliminating already calculated points
     sortbarObj(i,:) = NaN;
end

% Plotting barriers
barPlot = vertcat(barCoords{:});
plot(barPlot(:,1), barPlot(:,2), 'oy', 'DisplayName', 'Barrier Detections')

% Calculating centroids, and checking for moving vehicles 
% by seeing distance between detection points and its centroid
subplot(2, 3, 5)

for i = 1:length(carCoords)
    carCent(end+1,:) = mean(carCoords{i});
    dist = sqrt((carCoords{i}(:,1) - carCent(end,1)).^2 + (carCoords{i}(:,2) - carCent(end,2)).^2);
    mask = dist > carMov;
    if sum(mask) > 0
        carmovNum = carmovNum + 1;
        plot(carCent(end, 1), carCent(end, 2), 'og', 'DisplayName', 'Moving Car')
    else
        carStopCoords{end+1} = carCoords{i};
        carStopCent(end+1,:) = carCent(end,:);
        plot(carCent(end, 1), carCent(end, 2), 'or', 'DisplayName', 'Stopped Car')
    end
    
end

for i = 1:length(truCoords)
    truCent(end+1,:) = mean(truCoords{i});
    dist = sqrt((truCoords{i}(:,1) - truCent(end,1)).^2 + (truCoords{i}(:,2) - truCent(end,2)).^2);
    mask = dist > truMov;
    if sum(mask) > 0
        trumovNum = trumovNum + 1;
        plot(truCent(end, 1), truCent(end, 2), 'oy', 'DisplayName', 'Moving Truck')
    else
        truStopCoords{end+1} = truCoords{i};
        truStopCent(end+1,:) = truCent(end,:);
        plot(truCent(end, 1), truCent(end, 2), 'ob', 'DisplayName', 'Stopped Truck')
    end
end

% Calculating distances
subplot(2,3,6)

% Finding the index of the point closest to the first car
for i = 1:length(carStopCoords)   
    dist = sqrt((PP(:,1) - carStopCent(i,1)).^2 + (PP(:,2) - carStopCent(i,2)).^2);
    minDist = min(dist);
    minIdx = find(dist == minDist);
    if minIdx < mindistIdx
        mindistIdx = minIdx;
    end
end


for i = 1:length(truStopCoords)   
    dist = sqrt((PP(:,1) - truStopCent(i,1)).^2 + (PP(:,2) - truStopCent(i,2)).^2);
    minDist = min(dist);
    minIdx = find(dist == minDist);
    if minIdx < mindistIdx
        mindistIdx = minIdx;
    end
end


% After finding the index, calculating distance
for i = 1:length(PP(1:mindistIdx, 1))-1
    distDet(1, :) = PP(i);
    distDet(2, :) = PP(i+1);
    mindistCar =  mindistCar + pdist(distDet);
end

plot(PP(mindistIdx, 1), PP(mindistIdx, 2), 'or', 'DisplayName', 'LStopCar1')

% Finding the index of the point closest to the first and last barrier
barVect = vertcat(barCoords{:});
for i = 1:length(barVect)   
        dist = sqrt((PP(:,1) - barVect(i,1)).^2 + (PP(:,2) - barVect(i,2)).^2);
        minDist = min(dist);
        minIdx = find(dist == minDist);
        if minIdx < mindistFirstIdx
            mindistFirstIdx = minIdx;
        end
        if minIdx > maxdistLastIdx
            maxdistLastIdx = minIdx;
        end
end

% After finding the index, calculating distance
for i = 1:length(PP(1:mindistFirstIdx, 1))-1
    distDet(1, :) = PP(i);
    distDet(2, :) = PP(i+1);
    mindistFirstBar =  mindistFirstBar + pdist(distDet);
end

for i = 1:length(PP(1:maxdistLastIdx, 1))-1
    distDet(1, :) = PP(i);
    distDet(2, :) = PP(i+1);
    maxdistLastBar =  maxdistLastBar + pdist(distDet);
end

plot(PP(mindistFirstIdx, 1), PP(mindistFirstIdx, 2), 'og', 'DisplayName', 'LBarrFirst')
plot(PP(maxdistLastIdx, 1), PP(maxdistLastIdx, 2), 'ob', 'DisplayName', 'LBarrLast')

% Calculating final variables
clc
Lcar = IMSDist;
Peds = pedNum;
InPeds = 0;
StopCars = carNum + truNum - carmovNum - trumovNum;
MovCars = carmovNum + trumovNum;
Bikes = cycNum;
Lped1 = 0;
LStopCar1 = mindistCar;
LBarrFirst = mindistFirstBar;
LBarrLast = maxdistLastBar;

% Results

results = ['88939', string(Lcar), string(Peds), string(InPeds), string(StopCars), string(MovCars), string(Bikes) ...
    string(Lped1), string(LStopCar1), string(LBarrFirst), string(LBarrLast)];
results = join(results, ',');
file = fopen('TP1_results_88939.txt', 'wt');
fprintf(file, results);
fclose(file);


