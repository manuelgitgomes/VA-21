%% Exercise 1
clear, clc, close all

% Defining variables
WW = 150;
HH = 100;
P = [40, 80;
     85, 45;
     60, 20];
r = 4;
[XX, YY] = meshgrid(1:WW, 1:HH);

map = zeros (HH, WW);

% Placing the obstacles inside the map

for n = 1:height(P)
    x0 = P(n, 1);
    y0 = P(n, 2);
    mask = (XX - x0).^2 + (YY - y0).^2 <= r^2;
    map(mask) = 1;
end

% Showing the map
imshow(map)
title('Map')

% b)
% Calculating obstacles potential
Dmax=5; krep=100; maxPot=20;

Urep = zeros(size(map));

for x = 1:WW
    for y = 1:HH
        if (map(y, x) == 1)
            Urep(y, x) = maxPot;
            continue
        end
        mask = (XX - x).^2 + (YY - y).^2 <= Dmax^2;
        obst = mask & map;
        [xobs, yobs] = find(obst == 1);
        for i = 1:length(xobs)
            xob = xobs(i);
            yob = yobs(i);
            Urep(y, x) = Urep(y, x) + (1/2 * krep * (1 / norm([x - xob, y - yob]) - 1/Dmax)...
                .^2);
            if Urep(y, x) > maxPot
                Urep(y, x) = maxPot;
            end
        end
    end
end

% Showing the potentials
figure(2)
imshow(Urep)
title('Urep')

% c)
% Calculating destiny potential
Tx = 120;
Ty = 50;
katt=0.005;
Uatt = zeros(size(map));

for x = 1:WW
    for y = 1:HH
        Uatt(y, x) = 1/2 * katt * norm([x - Tx, y - Ty]).^2;
    end
end

% Total
U = Uatt + Urep;
figure(3)
surf(U)

% d)
%Plots
[Gx, Gy] = gradient(U);
Gx = -Gx; Gy= -Gy;
figure(4)
quiver(XX,YY, Gx, Gy, 5)
figure(5)
contour(U, 60)

% e)
% Calculate path
start = [15, 10];
path = stream2(XX, YY, Gx, Gy, start(:, 1), start(:, 2));
hold on
h = streamline(path);
h.Color = 'red';
h.LineWidth = 2;

% f)
B = ordfilt2(U,1,ones(3,3));
mm = (B==U);
[xmins, ymins] = find(mm);
plot(ymins, xmins,'ok')
