%% Exercise 1
clear, clc, close all
% Add path of toolbox
addpath(fullfile(matlabroot, 'toolbox','shared','tracking','fusionlib'));

% Define variables
initialDist = 150; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
finalDist = 1; % m

% Create scenario with data
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, ...
initialSpeed, brakeAccel, finalDist);

% Generate sensor
radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [20, 5], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 4, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

% Create scene demo
[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, ...
radarSensor); 

% Create structure to store sensor metrics
metrics = struct; 

% While demo is running
while advance(scenario) 
    % Obtain car pose
    gTruth = targetPoses(egoCar); 

    % Obtain time
    time = scenario.SimulationTime;

    % Obtain sensor detections
    [dets, ~, isValidTime] = radarSensor(gTruth, time); 

    % If the simulation is valid, update the demo GUI and the metrics
    % structure
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets);
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
    end
end

% Calculate RADAR errors and plotting
helperPlotSensorDemoDetections(metrics, 'position', 'reverse range', [-6 6]);
tgtCar = scenario.Actors(2);
rearOverhang = tgtCar.RearOverhang;
subplot(1,2,1);
hold on; plot(-rearOverhang*[1 1], ylim, 'k'); hold off;
legend('Error', '2\sigma noise', 'Rear overhang');

% Distância até ao alvo
radarRange = 30-(radarSensor.MountingLocation(1)+tgtCar.RearOverhang);
% Ângulo ocupado no radar pelo veículo a 30m
width = tgtCar.Width;
azSpan = rad2deg(width/radarRange);

%% Exercise 2
clear, clc, close all
% Create passing scenario
leadDist = 40; % m
speed = 50; % km/h
passSpeed = 70; % km/h
[scenario, egoCar] = helperCreateSensorDemoScenario('Passing', leadDist, ...
speed, passSpeed);

% Defining sensor
radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [20, 5], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 4, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

% Changing radar parameters
release(radarSensor);
radarSensor.HasRangeRate = true;
radarSensor.RangeRateResolution = 0.5;

restart(scenario);


% Create scene demo
[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, ...
radarSensor); 

% Create structure to store sensor metrics
metrics = struct; 

% While demo is running
while advance(scenario) 
    % Obtain car pose
    gTruth = targetPoses(egoCar); 

    % Obtain time
    time = scenario.SimulationTime;

    % Obtain sensor detections
    [dets, ~, isValidTime] = radarSensor(gTruth, time); 

    % If the simulation is valid, update the demo GUI and the metrics
    % structure
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets);
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
    end
end

% Calculate RADAR errors and plotting
helperPlotSensorDemoDetections(metrics, 'velocity', 'time', [-25 25]);
subplot(1,2,1);
legend('Lead car error', 'Lead car 2\sigma noise', ...
'Pass car error', 'Pass car 2\sigma noise', 'Location', 'northwest');

%% Exercise 3
clear, clc, close all
% Create scenario
initialDist = 150; % m
finalDist = 1; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
withPedestrian = true;
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, ...
initialSpeed, brakeAccel, finalDist, withPedestrian);

% Defining sensor
radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [20, 5], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 4, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

% Change radar parameters
release(radarSensor);radarSensor.ReferenceRange = 100; % m
radarSensor.ReferenceRCS = 0; % dBsm
radarSensor.DetectionProbability = 0.9;

% Create scene demo
[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, ...
radarSensor); 

% Create structure to store sensor metrics
metrics = struct; 

% While demo is running
while advance(scenario) 
    % Obtain car pose
    gTruth = targetPoses(egoCar); 

    % Obtain time
    time = scenario.SimulationTime;

    % Obtain sensor detections
    [dets, ~, isValidTime] = radarSensor(gTruth, time); 

    % If the simulation is valid, update the demo GUI and the metrics
    % structure
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets);
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
    end
end

% Plot SNR
helperPlotSensorDemoDetections(metrics, 'snr', 'range', [0 160]);
legend('Vehicle', 'Pedestrian');

%% Exercise 4
clear, clc, close all
% Create scenario
initialDist = 150; % m
finalDist = 1; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
withPedestrian = true;
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, ...
initialSpeed, brakeAccel, finalDist, withPedestrian);

% Defining sensor
radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [90, 10], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 10, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

% Change radar parameters
release(radarSensor);radarSensor.ReferenceRange = 50; % m
radarSensor.ReferenceRCS = 0; % dBsm
radarSensor.DetectionProbability = 0.9;

% Create scene demo
[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, ...
radarSensor); 

% Create structure to store sensor metrics
metrics = struct; 

% While demo is running
while advance(scenario) 
    % Obtain car pose
    gTruth = targetPoses(egoCar); 

    % Obtain time
    time = scenario.SimulationTime;

    % Obtain sensor detections
    [dets, ~, isValidTime] = radarSensor(gTruth, time); 

    % If the simulation is valid, update the demo GUI and the metrics
    % structure
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets);
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
    end
end

% Plot SNR
helperPlotSensorDemoDetections(metrics, 'snr', 'range', [0 160]);
legend('Vehicle', 'Pedestrian');

%% Exercise 5
clear, clc, close all
% Add path of toolbox
addpath(fullfile(matlabroot, 'toolbox','shared','tracking','fusionlib'));

% Define variables
initialDist = 150; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
finalDist = 1; % m
maxDistance = 10; %m

% Create scenario with data
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, ...
initialSpeed, brakeAccel, finalDist);
maxDistance = maxDistance + egoCar.Length - egoCar.RearOverhang;

% Generate sensor
radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [20, 5], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 4, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

% Create scene demo
[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, ...
radarSensor); 

% Create structure to store sensor metrics
metrics = struct; 

% While demo is running
while advance(scenario) 
    % Obtain car pose
    gTruth = targetPoses(egoCar); 

    % Obtain time
    time = scenario.SimulationTime;

    % Obtain sensor detections
    [dets, ~, isValidTime] = radarSensor(gTruth, time); 

    % If the simulation is valid, update the demo GUI and the metrics
    % structure
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets);
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
        if ~isempty(dets)
            distCoor = dets{1,1}.Measurement(1:3,:);
            dist = sqrt(sum(distCoor.^2));
            if dist < maxDistance
                disp('Danger, object close!')
            end
        end
    end
end

%% Exercise 6
clear, clc, close all
addpath(fullfile(matlabroot, 'examples', 'driving_radar_fusion', 'main'));

% Create the scenario
[scenario, egoVehicle, sensors] = helperCreateMultipathDrivingScenario;

% Load the recorded data
load('MultiPathRadarScenarioRecording.mat','detectionLog','configurationLog');

% Configuration of the sensors from the recording to set up the tracker
[~, sensorConfigurations] = ...
helperAssembleData(detectionLog{1},configurationLog{1});

for i = 1:numel(sensorConfigurations)
    sensorConfigurations{i}.FilterInitializationFcn = @helperInitGGIWFilter;
    sensorConfigurations{i}.SensorTransformFcn = @ctmeas;
end

tracker = trackerPHD('SensorConfigurations', sensorConfigurations,...
'PartitioningFcn',@(x)helperMultipathExamplePartitionFcn(x,2,5),...
'AssignmentThreshold',450,...
'ExtractionThreshold',0.8,...
'ConfirmationThreshold',0.85,...
'MergingThreshold',25,...
'DeletionThreshold',1e-2,...
'BirthRate',1e-2,...
'HasSensorConfigurationsInput',true...
);

% Create trackGOSPAMetric object to calculate GOSPA metric
gospaMetric = trackGOSPAMetric('Distance','custom', ...
'DistanceFcn',@helperGOSPADistance, ...
'CutoffDistance',35);

% Create display for visualization of results
display = helperMultiPathTrackingDisplay;

% Predicted track list for ghost filtering
predictedTracks = objectTrack.empty(0,1);

% Confusion matrix
confMat = zeros(5,5,numel(detectionLog));

% GOSPA metric
gospa = zeros(4,numel(detectionLog));

% Ground truth
groundTruth = scenario.Actors(2:end);

for i = 1:numel(detectionLog)
    % Advance scene for visualization of ground truth
    advance(scenario);
    
    % Current time
    time = scenario.SimulationTime;

    % Detections and sensor configurations
    [detections, configurations] = ...
    helperAssembleData(detectionLog{i},configurationLog{i});

    % Predict confirmed tracks to current time for classifying ghosts
    if isLocked(tracker)
        predictedTracks = predictTracksToTime(tracker,'confirmed',time);
    end

    % Classify radar detections as targets, ghosts, or static environment
    [targets, ghostStatic, ghostDynamic, static, reflectors, classificationInfo] ...
    = helperClassifyRadarDetections(detections, egoVehicle, predictedTracks);

    % Pass detections from target and sensor configurations to the tracker
    confirmedTracks = tracker(targets, configurations, time);

    % Visualize the results
    display(egoVehicle, sensors, targets, confirmedTracks, ghostStatic, ...
    ghostDynamic, static, reflectors);

    % Calculate GOSPA metric
    [gospa(1, i),~,~,gospa(2,i),gospa(3,i),gospa(4,i)] = ...
    gospaMetric(confirmedTracks, groundTruth);

    % Get true classification information and generate confusion matrix
    trueClassificationInfo = helperTrueClassificationInfo(detections);
    confMat(:,:,i) = helperConfusionMatrix(trueClassificationInfo, ...
    classificationInfo);
end
