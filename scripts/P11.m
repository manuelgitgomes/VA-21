%% Exercise 1
clear, clc, close all

waypoints = [-6.0,  16.2;
             26.9,  18.2;
             37.7,  19.1;
             51.2,  17.2;
             60.6,   8.1;
             66.6, -21.8];
laneWidth = 6;
speedLimit = 15;
lineHandles = [];

scenario = P11data;
refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath);
scenario.SampleTime = connector.TimeResolution;
replanRate = 1;
stepPerUpdate = (1/replanRate)/scenario.SampleTime;
viewer = P11data;
viewer.SampleTime = scenario.SampleTime;
carLen = viewer.Actors(1).Length;

chasePlot(viewer.Actors(1),'ViewLocation',-[carLen*3,0], ...
    'ViewHeight',10,'ViewPitch',20);
egoState = frenet2global(refPath, [0 0 0 laneWidth*3 0 0]);
terminalState = [nan speedLimit 0 laneWidth/2 0 0];
time = 3;

tic
isRunning = true;
while isRunning
    egoFrenetState = global2frenet(refPath,egoState); 
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,terminalState,time);
    optimalTrajectory = globalTraj(1).Trajectory; 
%     lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,1,1);
    for i = (2+(0:(stepPerUpdate-1)))
        % Approximate realtime visualization.
        dt = toc;
        if scenario.SampleTime-dt > 0
            pause(scenario.SampleTime-dt);
        end
        egoState = optimalTrajectory(i,:); %Percorrer os passos da trajetória
        %Atualizar o estado do veículo
        viewer.Actors(1).Position(1:2) = egoState(1:2);
        viewer.Actors(1).Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
        viewer.Actors(1).Yaw = egoState(3)*180/pi;
        viewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
        % Update driving scenario.
        isRunning = advance(viewer);
        tic;
    end

end

%% Exercise 2
clear, clc, close all

waypoints = [-6.0,  16.2;
             26.9,  18.2;
             37.7,  19.1;
             51.2,  17.2;
             60.6,   8.1;
             66.6, -21.8];
laneWidth = 6;
speedLimit = 15;
lineHandles = [];

scenario = P11data2;
refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath);
scenario.SampleTime = connector.TimeResolution;
replanRate = 1;
stepPerUpdate = (1/replanRate)/scenario.SampleTime;
viewer = P11data2;
viewer.SampleTime = scenario.SampleTime;
carLen = viewer.Actors(1).Length;
carWidth = viewer.Actors(1).Width;

chasePlot(viewer.Actors(1),'ViewLocation',-[carLen*3,0], ...
    'ViewHeight',10,'ViewPitch',20);
egoState = frenet2global(refPath, [0 0 0 laneWidth*3 0 0]);
terminalState = [nan speedLimit 0 laneWidth/2 0 0];
time = 4;
maxTimeHorizon = max(time);

tic
isRunning = true;

% Exercise 2 per se
capList = dynamicCapsuleList;
egoID = 1;
[egoID, egoGeom] = egoGeometry(capList, egoID);
egoGeom.Geometry.Length = carLen;
egoGeom.Geometry.Radius = carWidth/2;
updateEgoGeometry(capList, egoID, egoGeom);

actorID = 2;
actorGeom = egoGeom;
updateObstacleGeometry(capList, actorID, actorGeom);

numActors = numel(actorID);
futureTrajectory = repelem(struct('Trajectory',[]),numActors,1);

[actorID, actorPoses] = obstaclePose(capList, capList.ObstacleIDs);

while isRunning
    [curActorState,futureTrajectory,isRunning] = ...
    exampleHelperRetrieveActorGroundTruth(scenario, futureTrajectory, ...
    replanRate, maxTimeHorizon);
    
    egoFrenetState = global2frenet(refPath,egoState); 
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,terminalState,time);
    optimalTrajectory = globalTraj(1).Trajectory;
    lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,1,1);


    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);
    
    egoPoses.States = optimalTrajectory(:,1:3);
    updateEgoPose(capList,egoID,egoPoses);
    hold on;
    show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1); %Desenhar as capsulas
    hold off;
    
    isColliding = checkCollision(capList);
    
    if all(~isColliding)    
        % Nenhuma colisão detetada
    else
        disp('Colisão detetada !!!')
        break
    end

    for i = (2+(0:(stepPerUpdate-1)))
        % Approximate realtime visualization.
        dt = toc;
        if scenario.SampleTime-dt > 0
            pause(scenario.SampleTime-dt);
        end
        egoState = optimalTrajectory(i,:); %Percorrer os passos da trajetória
        %Atualizar o estado do veículo
        viewer.Actors(1).Position(1:2) = egoState(1:2);
        viewer.Actors(1).Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
        viewer.Actors(1).Yaw = egoState(3)*180/pi;
        viewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
        % Update driving scenario.
        isRunning = advance(viewer);
        tic;
    end
end

%% Exercise 3
addpath \home\nel\Documents\MATLAB\Examples\R2021b\autonomous_control\HighwayTrajectoryPlanningUsingFrenetReferencePathExample
scenario = drivingScenarioTrafficExample; %Novo cenário
actorID = (1:5)'; %Neste cenário existem 5 atores
actorGeom = repelem(egoGeom, 5, 1);
updateObstacleGeometry(capList, actorID, actorGeom)