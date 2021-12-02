%% Exercise 1
clear, clc, close all

addpath('Documents/VA-21/scripts/')

coords=[
40.62940, -8.6582, 0 %começa no DEM
40.63012, -8.6592, 0 %vira para rua dos lavadoiros
40.62956, -8.6599, 0 %vira para rua da pega
40.63050, -8.6611, 0 %fim da rua
];

% Plotting the coordinates
figure(1)
geoplot(coords(:,1), coords(:,2), '-*b')

% Converting from global coordinates to local
origin = [coords(1,1), coords(1,2), coords(1,3)];
[xEast, yNorth, zUp] = latlon2local(coords(:,1), coords(:,2), ...
    coords(:,3), origin);

% Creating driving scenario
s = drivingScenario('GeoReference',origin);
v = vehicle(s);
waypoints = [xEast, yNorth, zUp];

% Create new waypoints
for i = 1:3
    waypoints = AddMidWaypoints(waypoints);
end

% Defining speed
speed = zeros(length(waypoints), 1);

for i = 1:length(speed)
    if rem(i,4) == 0
        speed(i) = 15;
    else
        speed(i) = 15;
    end
end

smoothTrajectory(v,waypoints,speed);

% =============== Para a IMU ==== (Acelerómetrro e Giroscópio)
mountingLocationIMU = [1 2 3];
mountingAnglesIMU =   [0 0 0];
% Converte a orientação de Euler para quaternion para o simulador.
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');
% Adiciona o sensor de IMU (ReferenceFrame tem de ser ENU.)
myimu = imuSensor('SampleRate',1/s.SampleTime,'ReferenceFrame','ENU');

% =============== Para o GPS ====
mountingLocationGPS = [1 2 3];
mountingAnglesGPS = [50 40 30];
% Converte a orientação de Euler para quaternion para o simulador.
orientVeh2GPS = quaternion(mountingAnglesGPS,'eulerd','ZYX','frame');
% Adiciona o sensor de GPS (ReferenceFrame tem de ser ENU.)
mygps = gpsSensor('ReferenceLocation',origin,'ReferenceFrame','ENU');

% =============== Para o Encoder (Hodometria) ====
myencoder = wheelEncoderAckermann('TrackWidth',v.Width,...
'WheelBase',v.Wheelbase,'SampleRate',1/s.SampleTime);

% Exercise 2
% Leituras da IMU.
accel = []; % [ax, ay, az]
gyro = []; % [wx, wy, wz]
% Leituras dos 4 encoders das 4 rodas
ticks = []; % [bl, br, fl, fr]  % f/b front-back, r/l right-left
% Leituras do GPS.
gpsPos = []; % [lat, lon, alt]
gpsVel = []; % [vx, vy, vz]
% Taxa do GPS em função da taxa de simulação
simSamplesPerGPS = (1/s.SampleTime)/mygps.SampleRate;
% gps.SampleRate é por defeito 1 s, mas pode ser alterado
% s.SampleTime é por defeito 0.01 s, mas pode ser alterado

gtPos = [];

idx = 0;

restart(s)

while advance(s)
    groundTruth = state(v);

    % decompõe a estrutura do ground truth com certas conversões
    posVeh    = groundTruth.Position;
    orientVeh = quaternion( fliplr(groundTruth.Orientation), ...
    'eulerd','ZYX', 'frame');
    velVeh    = groundTruth.Velocity;
    accVeh    = groundTruth.Acceleration;
    angvelVeh = deg2rad(groundTruth.AngularVelocity);

    % Converte grandezas de movimento do referencial do veículo para a IMU
    [posIMU,orientIMU,velIMU,accIMU,angvelIMU] = ...
    transformMotion( mountingLocationIMU,orientVeh2IMU, ...
    posVeh,orientVeh,velVeh,accVeh,angvelVeh);

    % Lê e acumula mais uma entrada de valores de acelerações e giroscópio
    [accel(end+1,:), gyro(end+1,:)]= myimu(accIMU,angvelIMU,orientIMU);
    
    % Lê e acumula mais um valor de ticks dos encoders
    ticks(end+1,:) = myencoder(velVeh, angvelVeh, orientVeh);


    % Lê e acumula mais uma entrada de valores de coordenadas e velocidade
    if ( mod(idx, simSamplesPerGPS) == 0)
        % Converte grandezas de movimento do referencial do veículo para o GPS
        [posGPS,orientGPS,velGPS,accGPS,angvelGPS] = ...
        transformMotion( mountingLocationGPS, ...
        orientVeh2GPS,posVeh,orientVeh,velVeh,accVeh,angvelVeh);
        [gpsPos(end+1,:), gpsVel(end+1,:)]  = mygps(posGPS,velGPS);
    end
    idx = idx + 1;

    [gtPos(end+1, :)] = posVeh;
end

figure
subplot(2,3,1)
plot(waypoints(:,1),waypoints(:,2),'-o')
title('Waypoints')

subplot(2,3,2)
hold on
plot(find(ticks(:,1)), ticks(:, 1), '-r')
plot(find(ticks(:,2)), ticks(:, 2), '-g')
plot(find(ticks(:,3)), ticks(:, 3), '-b')
plot(find(ticks(:,4)), ticks(:, 4), '-y')
title('Wheel Encoder')

subplot(2,3,3)
hold on
plot(find(accel(:,1)), accel(:, 1), '-r')
plot(find(accel(:,2)), accel(:, 2), '-g')
plot(find(accel(:,3)), accel(:, 3), '-b')
title('Accelerometer')

subplot(2,3,4)
hold on
plot(find(gyro(:,1) < 10), gyro(:, 1), '-r')
plot(find(gyro(:,2) < 10), gyro(:, 2), '-g')
plot(find(gyro(:,3) < 10), gyro(:, 3), '-b')
title('Accelerometer')

subplot(2,3,5)
geoplot(gpsPos(:,1), gpsPos(:,2), '-b')
title('GPS Position')

subplot(2,3,6)
hold on
plot(find(gpsVel(:,1)), gpsVel(:, 1), '-r')
plot(find(gpsVel(:,2)), gpsVel(:, 2), '-g')
plot(find(gpsVel(:,3)), gpsVel(:, 3), '-b')
title('GPS Velocity')


