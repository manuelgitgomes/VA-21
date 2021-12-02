function newW=AddMidWaypoints(W)

% Create new W
[lins, cols] =  size(W);
newW = zeros([2*lins-1, cols]);

% Add waypoints and midpoints
for i = 1:lins-1
    newW(2*i-1, :) = W(i, :);
    newW(2*i, :) = (W(i+1,:) + W(i, :))/2;
end

% Add last waypoint
newW(2*lins-1, :) = W(lins, :);