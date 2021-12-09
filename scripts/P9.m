%% Exercise 1
clear, clc, close all
load("office_area_gridmap.mat","occGrid")
show(occGrid)

% Set start and goal poses.
start = [-1.0,0.0,-pi];
goal = [14,-2.25,0];

% Show start and goal positions of robot.
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')

% Show start and goal headings.
r = 0.5;
plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'r-')
plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'m-')
hold off

% Defining vehicle
bounds = [occGrid.XWorldLimits; occGrid.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

% Planning map
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = occGrid;
stateValidator.ValidationDistance = 0.05;
planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 30000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
rng default

% Planning
[pthObj, solnInfo] = plan(planner,start,goal);
show(occGrid)
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

% Interpolate and plot path.
plot(pthObj.States(:,1),pthObj.States(:,2),'G-','LineWidth',4)
interpolate(pthObj,300)
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

% Show start and goal in grid map.
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')
hold off

% Only turns left
% Only making left turns
goLeft = true;

% Create the state space
ssCustom = ExampleHelperStateSpaceOneSidedDubins(bounds,goLeft);
ssCustom.MinTurningRadius = 0.4;

% Planning map
stateValidator = validatorOccupancyMap(ssCustom); 
stateValidator.Map = occGrid;
stateValidator.ValidationDistance = 0.05;
planner = plannerRRT(ssCustom,stateValidator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 30000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
rng default

% Planning
figure(2)
[pthObj, solnInfo] = plan(planner,start,goal);
show(occGrid)
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

% Interpolate and plot path.
plot(pthObj.States(:,1),pthObj.States(:,2),'G-','LineWidth',4)
interpolate(pthObj,300)
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

%% Exercise 2
clear, clc, close all
load("office_area_gridmap.mat","occGrid")
show(occGrid)

% Add obstacles
L = 0.75;
step = 1/occGrid.Resolution;
P1 = [2, 4.5];
P2 = [2, -4.5];
[I1, J1] = meshgrid((P1(1)-L/2):step:(P1(1)+L/2), (P1(2)-L/2):step:(P1(2)+L/2));
setOccupancy(occGrid, [I1(:) J1(:)], 1)
[I2, J2] = meshgrid((P2(1)-L/2):step:(P2(1)+L/2), (P2(2)-L/2):step:(P2(2)+L/2));
setOccupancy(occGrid, [I2(:) J2(:)], 1)

% Inflating
occGridOriginal = copy(occGrid);
inflate(occGrid, 0.1)

% Set start and goal poses.
start = [-1.0,0.0,-pi];
goal = [14,-2.25,0];

% Show start and goal positions of robot.
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')

% Show start and goal headings.
r = 0.5;
plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'r-')
plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'m-')
hold off

% Defining vehicle
bounds = [occGrid.XWorldLimits; occGrid.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

% Planning map
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = occGrid;
stateValidator.ValidationDistance = 0.05;
planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 10;
planner.MaxIterations = 30000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
% rng default
rng(3)

% Planning
[pthObj, solnInfo] = plan(planner,start,goal);
show(occGridOriginal)
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

% Interpolate and plot path.
plot(pthObj.States(:,1),pthObj.States(:,2),'G-','LineWidth',4)
interpolate(pthObj,300)
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

% Show start and goal in grid map.
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')
hold off

figure(2)
show(occGrid)

%% Exercise 4
clear, clc, close all
map = binaryOccupancyMap(100, 80, 1);
occ = zeros(80, 100);

occ(1,:) = 1;
occ(end,:) = 1;
occ([1:30, 51:80],1) = 1;
occ([1:30, 51:80],end) = 1;

occ(40,20:80) = 1;
occ(28:52,[20:21 32:33 44:45 56:57 68:69 80:81]) = 1;

occ(1:12, [20:21 32:33 44:45 56:57 68:69 80:81]) = 1;

occ(end-12:end, [20:21 32:33 44:45 56:57 68:69 80:81]) = 1;

setOccupancy(map, occ)

figure
show(map)
title('Warehouse Floor Plan')

% Create map that will be updated with sensor readings
estMap = occupancyMap(occupancyMatrix(map));

% Create a map that will inflate the estimate for planning
inflateMap = occupancyMap(estMap);

vMap = validatorOccupancyMap;
vMap.Map = inflateMap;
vMap.ValidationDistance = .1;
planner = plannerHybridAStar(vMap, 'MinTurningRadius', 4);

entrance = [1 40 0];
packagePickupLocation = [63 44 -pi/2];
route = plan(planner, entrance, packagePickupLocation);
route = route.States;

% Get poses from the route.
rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
startPoses = route(1:end-1,:);
endPoses = route(2:end,:);

rsPathSegs = connect(rsConn, startPoses, endPoses);
poses = [];
for i = 1:numel(rsPathSegs)
    lengths = 0:0.1:rsPathSegs{i}.Length;
    [pose, ~] = interpolate(rsPathSegs{i}, lengths);
    poses = [poses; pose];
end

figure
show(planner)
title('Initial Route to Package')

obstacleWidth = 6;
obstacleHeight = 11;
obstacleBottomLeftLocation = [34 49];
values = ones(obstacleHeight, obstacleWidth);
setOccupancy(map, obstacleBottomLeftLocation, values)
obstacleBottomLeftLocation = [54 49];
values = ones(obstacleHeight, obstacleWidth);
setOccupancy(map, obstacleBottomLeftLocation, values)

figure 
show(map)
title('Warehouse Floor Plan with Obstacle')

rangefinder = rangeSensor('HorizontalAngle', pi/3);
numReadings = rangefinder.NumReadings;

% Setup visualization.
helperViz = HelperUtils;
figure
show(inflateMap);
hold on
robotPatch = helperViz.plotRobot(gca, poses(1,:));
rangesLine = helperViz.plotScan(gca, poses(1,:), ...
    NaN(numReadings,1), ones(numReadings,1));
axesColors = get(gca, 'ColorOrder');
routeLine = helperViz.plotRoute(gca, route, axesColors(2,:));
traveledLine = plot(gca, NaN, NaN);
title('Forklift Route to Package')
hold off

idx = 1;
tic;
while idx <= size(poses,1)
    % Insert range finder readings into estimated map.
    [ranges, angles] = rangefinder(poses(idx,:), map);
    insertRay(estMap, poses(idx,:), ranges, angles, ...
        rangefinder.Range(end));
    
    % Reset and reinflate planning map
    setOccupancy(inflateMap, getOccupancy(estMap));
    inflate(inflateMap,1,'grid');

    % Update visualization.
    show(inflateMap, 'FastUpdate', true);
    helperViz.updateWorldMap(robotPatch, rangesLine, traveledLine, ...
        poses(idx,:), ranges, angles)
    drawnow
    
    % Regenerate route when obstacles are detected in the current one.
    isRouteOccupied = any(checkOccupancy(inflateMap, poses(:,1:2)));
    if isRouteOccupied && (toc > 0.1)
        % Calculate new route.
        route = plan(planner, poses(idx,:), packagePickupLocation);
        route = route.States;
        
        % Get poses from the route.
        startPoses = route(1:end-1,:);
        endPoses = route(2:end,:);
        rsPathSegs = connect(rsConn, startPoses, endPoses);
        poses = [];
        for i = 1:numel(rsPathSegs)
            lengths = 0:0.1:rsPathSegs{i}.Length;
            [pose, ~] = interpolate(rsPathSegs{i}, lengths);
            poses = [poses; pose]; %#ok<AGROW>
        end
        
        routeLine = helperViz.updateRoute(routeLine, route);
        idx = 1;
        tic;
    else
        idx = idx + 1;
    end
end