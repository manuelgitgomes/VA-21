%% Exercício 1
clear, clc, close all
load('P1data.mat');
n = numel(sns1);
posx = zeros(n,1);
posy = posx;

for k = 1:n
    posx(k) = sns1(k).ActorPoses(1).Position(1);
    posy(k) = sns1(k).ActorPoses(1).Position(2);
end
figure(1)
title('Position')
plot(posx, posy, 'LineWidth',4)

%% Exercício 1 (alternative)
clear, clc, close all
load('P1data.mat');
t = [sns1.Time];
PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', sns1, 'UniformOutput', false))';
figure(1)
title('Position')
subplot(3,1,1)
plot(t, PP)
legend('x','y','z')
xlabel('t (s)')
VV = cell2mat(arrayfun(@(S) S.ActorPoses(1).Velocity', sns1, 'UniformOutput', false))';
subplot(3,1,2)
title('Velocity')
plot(t, VV)
legend('v_x','v_y','v_z')
xlabel('t (s)')
WW = cell2mat(arrayfun(@(S) S.ActorPoses(1).AngularVelocity', sns1, 'UniformOutput', false))';
subplot(3,1,3)
title('Angular Velocity')
plot(t, VV)
legend('\omega_x','\omega_y','\omega_z')
xlabel('t (s)')


%% Exercicio 2
clear, clc, close all
load('P1data.mat');
t = [sns1.Time];
PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', sns1, 'UniformOutput', false))';
VV = cell2mat(arrayfun(@(S) S.ActorPoses(1).Velocity', sns1, 'UniformOutput', false))';
VP = zeros(numel(VV(:,1))-1, 3);
for i = 1:(numel(PP(:,1))-1)
     VP(i,:) = (PP(i+1,:) - PP(i,:)) ./ (t(i+1) - t(i));
end
DV = VP-VV(1:end-1,:);


figure(1)
subplot(2,2,1)
plot(t,PP)
title('Position')
legend('x','y','z')
xlabel('t (s)')
subplot(2,2,2)
plot(t,VV)
title('Velocity')
legend('v_x','v_y','v_z')
xlabel('t (s)')
subplot(2,2,3)
plot(t(1:end-1),VP)
title('Calculated Velocity')
legend('v_x','v_y','v_z')
xlabel('t (s)')
subplot(2,2,4)
plot(t(1:end-1),DV)
title('Difference in Velocity')
legend('v_x','v_y','v_z')
xlabel('t (s)')

%% Exercicio 3
clear, clc, close all
load('P1data.mat');
t = [sns1.Time];
PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', sns1, 'UniformOutput', false))';
plot(PP(:,1), PP(:,2))
hold on
for n=1:numel(sns1)
    objs=sns1(n).ObjectDetections;
    posCar=PP(n,:);
    orCar=[sns1(n).ActorPoses(1).Yaw sns1(n).ActorPoses(1).Pitch sns1(n).ActorPoses(1).Roll]*pi/180;
    TCtrans = trvec2tform(posCar);
    TCrot = eul2tform(orCar);
    TCar = TCtrans * TCrot;
    for i=1:numel(objs)
        posObj = objs{i}.Measurement(1:3)';
        orObj = (objs{i}.Measurement(4:6))' * pi/180;
        TOtrans = trvec2tform(posObj);
        TOrot = eul2tform(orObj);
        TObj = TOtrans * TOrot;
        Pmundo = TCar * TObj * [0, 0, 0, -1]';
        plot(Pmundo(1,:), Pmundo(2,:), 'o')
    end
end
% posCar=PP(n,:);
% orCar=[sns1(n).ActorPoses(1).Yaw sns1(n).ActorPoses(1).Pitch sns1(n).ActorPoses(1).Roll]*pi/180;
% TCtrans = trvec2tform(posCar);
% TCrot = eul2tform(orCar);
% TCar = TCtrans * TCrot;


%% Exercise 4
clear, clc, close all
load('P1datacam.mat');
t = [sns2.Time];
PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', sns2, 'UniformOutput', false))';
plot(PP(:,1), PP(:,2))
hold on
for n=1:numel(sns2)
    objs=sns2(n).ObjectDetections;
    posCar=PP(n,:);
    orCar=[sns2(n).ActorPoses(1).Yaw sns2(n).ActorPoses(1).Pitch sns2(n).ActorPoses(1).Roll]*pi/180;
    TCtrans = trvec2tform(posCar);
    TCrot = eul2tform(orCar);
    TCar = TCtrans * TCrot;
    for i=1:numel(objs)
        posObj = objs{i}.Measurement(1:3)';
        orObj = (objs{i}.Measurement(4:6))' * pi/180;
        TOtrans = trvec2tform(posObj);
        TOrot = eul2tform(orObj);
        TObj = TOtrans * TOrot;
        Pmundo = TCar * TObj * [0, 0, 0, -1]';
        plot(Pmundo(1,:), Pmundo(2,:), 'o')
    end
end
