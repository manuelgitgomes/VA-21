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
barNum = 0;
posDet = zeros(2, 2);
objDist = 3;
count = false;
object = false;
PPAPDist = 0;
IMSDist = 0;
interval = 1;
ppapDet = zeros(2, 2);
imsDet = zeros(2, 2);
counter = 0;
unused = [];
carCoords = {};
cycCoords = {};
pedCoords = {};
barCoords = {};

% Load data
[allData, scenario, sensors] = scene1();

% Erasing Actor Pose
for n=1:numel(allData)
    allData(n).ActorPoses(2:end)=[];
end

% Defining time
t = [allData.Time];

% Plotting ego position with Actor Poses
PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', allData, 'UniformOutput', false))';

% Plotting ego position with IMS Measurements
% PP = cell2mat(arrayfun(@(S) S.INSMeasurements{1}.Position', allData, 'UniformOutput', false))';

% Calculating distance travelled
% % Using Actor Poses
% for i = 1:length(PPAP)-1
%     ppapDet(1, :) = PPAP(i);
%     ppapDet(2, :) = PPAP(i+1);
%     PPAPDist = PPAPDist + pdist(ppapDet);
% end

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
plot(carObj(:,1), carObj(:,2), 'or', 'DisplayName', 'Vehicle Detections')
plot(cycObj(:,1), cycObj(:,2), 'og', 'DisplayName', 'Bicycle Detections')
plot(pedObj(:,1), pedObj(:,2), 'ob', 'DisplayName', 'Pedestrian Detections')

% Order matrixes
sortcarObj = carObj(:, [1, 2]);
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
            sortpedObj(minIdx, :) = sortpedObj(i+1, :);
            sortpedObj(i+1, :) = v;
            if ~isempty(unused)
                [~,index] = ismember(unused, v,'rows');
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
        str = strcat('Found ', int2str(num), ' bars nearby, after backtracking.');
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

