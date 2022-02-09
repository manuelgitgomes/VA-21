%% TP2
clear, clc, close all
addpath("TP2/")
[~, scenario, ~] = TP2_Cenario_2();
% [~, scenario, ~] = DSD();

[sensors, ~] = createSensors(scenario);
egoVehicle = scenario.Actors(1);

% Collision check time stamps
tSteps = 0.5:0.5:5;

scenario.SampleTime = 0.1;

capList = dynamicCapsuleList;
capList.MaxNumSteps = 1+floor(max(tSteps)/scenario.SampleTime);
egoID = 1;
rearAxleRatio = 0.25;
carLen = scenario.Actors(egoID).Length;
carWidth = scenario.Actors(egoID).Width;
laneWidth = 3.6;

[egoID, egoGeom] = egoGeometry(capList,egoID);

% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

% Initial state of the ego vehicle
refPath = helperGetReferencePath();
egoState = frenet2global(refPath,[0 0 0 -0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Defining tracker 
tracker = trackerJPDA('FilterInitializationFcn',@helperInitRefPathFilter,...
    'AssignmentThreshold',[200 inf],...
    'ConfirmationThreshold',[4 5],...
    'DeletionThreshold',[5 5]);

while advance(scenario)
    % Current time
    time = scenario.SimulationTime;
    % Obter as deteções
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento 
    if numel(detections) > 0
        detections{1}
    end
    % de medições para compatibilização e parametrização de erros
    tracks = tracker(detections, time);


    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end

    % Planear a trajetória
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPoseFrenet);
    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList, ...
        currActorState, egoState); %Último exercicio da aula anterior
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    % Mover o veiculo para a posição atual
    egoState = optimalTrajectory(2,:)
    moveEgoToState(egoVehicle,egoState);
end