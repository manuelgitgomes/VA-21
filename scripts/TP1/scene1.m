function [allData, scenario, sensors] = scene1()
%scene1 - Returns sensor detections
%    allData = scene1 returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = scene1 optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.11 (R2021b) and Automated Driving Toolbox 3.4 (R2021b).
% Generated on: 01-Dec-2021 14:38:42

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {}, 'INSMeasurements', {});
running = true;
while running

    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    % Get the state of the ego vehicle
    actorState = state(egoVehicle);
    time  = scenario.SimulationTime;

    objectDetections = {};
    laneDetections   = [];
    ptClouds = {};
    insMeas = {};
    isValidTime = false(1, numSensors);
    isValidLaneTime = false(1, numSensors);
    isValidPointCloudTime = false(1, numSensors);
    isValidINSTime = false(1, numSensors);

    % Generate detections for each sensor
    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        % Generate the ego vehicle lane boundaries
        if isa(sensor, 'visionDetectionGenerator')
            maxLaneDetectionRange = min(500,sensor.MaxRange);
            lanes = laneBoundaries(egoVehicle, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
        end
        type = getDetectorOutput(sensor);
        if strcmp(type, 'Objects only')
            [objectDets, numObjects, isValidTime(sensorIndex)] = sensor(poses, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes only')
            [laneDets, ~, isValidTime(sensorIndex)] = sensor(lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes and objects')
            [objectDets, numObjects, isValidTime(sensorIndex), laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes with occlusion')
            [laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'PointCloud')
            if sensor.HasRoadsInputPort
                rdmesh = roadMesh(egoVehicle,min(500,sensor.MaxRange));
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, rdmesh, time);
            else
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, time);
            end
            ptClouds = [ptClouds; ptCloud]; %#ok<AGROW>
        elseif strcmp(type, 'INSMeasurement')
            insMeasCurrent = sensor(actorState, time);
            insMeas = [insMeas; insMeasCurrent]; %#ok<AGROW>
            isValidINSTime(sensorIndex) = true;
        end
    end

    % Aggregate all detections into a structure for later use
    if any(isValidTime) || any(isValidLaneTime) || any(isValidPointCloudTime) || any(isValidINSTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections', {laneDetections}, ...
            'PointClouds',   {ptClouds}, ... %#ok<AGROW>
            'INSMeasurements',   {insMeas}); %#ok<AGROW>
    end

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release all the sensor objects so they can be used again.
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = insSensor('TimeInput', true, ...
    'MountingLocation', [0.95 0 0]);
sensors{2} = lidarPointCloudGenerator('SensorIndex', 2, ...
    'SensorLocation', [0.95 0], ...
    'ActorProfiles', profiles);
sensors{3} = drivingRadarDataGenerator('SensorIndex', 3, ...
    'MountingLocation', [3.7 0 0.2], ...
    'RangeLimits', [0 20], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [60 5], ...
    'Profiles', profiles);
sensors{4} = visionDetectionGenerator('SensorIndex', 4, ...
    'SensorLocation', [3.7 0], ...
    'MaxRange', 50, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1814.81018227767 1814.81018227767],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
numSensors = 4;

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [91.9 47.3 0;
    77.1 55.9 0;
    65.1 51.2 0;
    54.8 42.5 0;
    52.4 31.1 0;
    53 19.2 0;
    56.9 13.2 0;
    65.6 5.7 0;
    71.8 -3.8 0;
    69.6 -11.4 0;
    65.1 -19.4 0;
    57.3 -33.6 0;
    39.9 -29.2 0;
    31.3 -28.4 0;
    19.8 -25.5 0;
    8.2 -22.6 0];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the barriers
barrierCenters = [95 53.4 0;
    86.8 60.8 0;
    75.9 61.6 0];
barrier(scenario, barrierCenters, ...
    'ClassID', 6, ...
    'Width', 0.433, ...
    'Mesh', driving.scenario.guardrailMesh, 'PlotColor', [0.55 0.55 0.55], 'Name', 'Guardrail');

barrierCenters = [69.3 12 0;
    75.5 6.5 0;
    78.3 -0.9 0;
    77 -11.8 0;
    67.1 -32.5 0;
    60.3 -42.6 0;
    53.4 -43.3 0;
    45.8 -39.7 0];
barrier(scenario, barrierCenters, ...
    'ClassID', 6, ...
    'Width', 0.433, ...
    'Mesh', driving.scenario.guardrailMesh, 'PlotColor', [0.55 0.55 0.55], 'Name', 'Guardrail1');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 4.848, ...
    'Width', 1.842, ...
    'Height', 1.517, ...
    'Position', [95.58 46.21 0], ...
    'RearOverhang', 1.119, ...
    'FrontOverhang', 0.911, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [95.58 46.21 0;
    90.67 51.25 0.01;
    85.57 55.5 0.01;
    79.82 57.67 0.01;
    74.72 57.39 0.01;
    68.68 55.88 0.01;
    63.5 53.14 0.01;
    59.15 50.03 0.01;
    56.04 46.91 0.01;
    51.98 39.17 0.01;
    50 32 0.01;
    49.91 22 0.01;
    51.98 14.17 0.01;
    56.51 9.83 0.01;
    61.42 6.34 0.01;
    65.95 2.09 0.01;
    69.22 -1.84 0.01;
    69.31 -7.68 0.01;
    65.82 -14.66 0.01;
    59.1 -30.28 0.01;
    55.69 -32.38 0.01;
    49.85 -31.24 0.01;
    44.09 -28.19 0.01;
    39.37 -27.05 0.01;
    31.87 -26.36 0.01;
    25.94 -25.22 0.01;
    20.44 -23.74 0.01;
    10.49 -21.03 0.01;
    8.66 -20.68 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30];
smoothTrajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 4.848, ...
    'Width', 1.842, ...
    'Height', 1.517, ...
    'Position', [42.2 29.9 0], ...
    'RearOverhang', 1.119, ...
    'FrontOverhang', 0.911, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 4.848, ...
    'Width', 1.842, ...
    'Height', 1.517, ...
    'Position', [61.5 -4.4 0], ...
    'RearOverhang', 1.119, ...
    'FrontOverhang', 0.911, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.59, ...
    'Width', 0.82, ...
    'Height', 1.7, ...
    'Position', [73.1 61.2 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.59, ...
    'Width', 0.82, ...
    'Height', 1.7, ...
    'Position', [57.1 53.5 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian1');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.59, ...
    'Width', 0.82, ...
    'Height', 1.7, ...
    'Position', [60.2 20 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian2');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.59, ...
    'Width', 0.82, ...
    'Height', 1.7, ...
    'Position', [11.3 -30 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian3');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.59, ...
    'Width', 0.82, ...
    'Height', 1.7, ...
    'Position', [53.3 -27.7 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian4');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.59, ...
    'Width', 0.82, ...
    'Height', 1.7, ...
    'Position', [61.4 -33.4 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian5');

actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.63, ...
    'Width', 0.55, ...
    'Height', 1.53, ...
    'Position', [61.4 36.2 0], ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'Name', 'Bicycle');

actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.63, ...
    'Width', 0.55, ...
    'Height', 1.53, ...
    'Position', [45.8 18.8 0], ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'Name', 'Bicycle1');

function output = getDetectorOutput(sensor)

if isa(sensor, 'visionDetectionGenerator')
    output = sensor.DetectorOutput;
elseif isa(sensor, 'lidarPointCloudGenerator')
    output = 'PointCloud';
elseif isa(sensor, 'insSensor')
    output = 'INSMeasurement';
else
    output = 'Objects only';
end

