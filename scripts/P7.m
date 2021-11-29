%% Exercise 1
clear, clc, close all

% Defining the function
n = 1000;
xs = linspace(-4*pi, 4*pi, n);
ys = cos(xs).*exp(-abs(xs)/5);
plot(xs, ys, 'g')

% Defining neural network
hN=10; %hidden neurons
vanet = feedforwardnet(hN);

% Defining inputs
ni = 100;
rr = randperm(n, ni);
x = xs(rr);
t = ys(rr);

% Visualizing the random points
hold on
plot(x,t,'ro')

% Starting the training
% vanet.divideParam.valRatio=0;
[vanet,tr] = train(vanet,x,t);

% Showing the final points
yf = vanet(xs);
norm(yf-ys)
plot(xs, yf, 'b')

%% Exercise 2
clear, clc, close all

% Calling the neural network
net = alexnet;

% Handle to read camera
cam = webcam(1);

% Show webcam image
img = snapshot(cam);
figure(1)
hold on
ims = imshow(img);
tt = title('Original');

% Plot center
% [h, w, c] = size(img);
% coords = [round((w - 277) / 2), round((h - 277) / 2), 277, 277];
% rectangle('Position', coords, 'EdgeColor', 'y')

% Infinite cycle
while 1
    % Getting webcam image, resizing and labeling
    img = snapshot(cam);
    imgR = imresize(img, [227 227]);
    label = classify(net,imgR);

    % Visualization 
    ims.CData = img;
    tt.String = label;
end

%% Exercise 3
clear, clc, close all

% Calling the neural network
net = googlenet;

% Handle to read camera
cam = webcam(1);

% Show webcam image
img = snapshot(cam);
h = figure(1);
h.Position(3) = 2*h.Position(3);
ax1 = subplot(1,2,1);
ims = imshow(img);
tt = title('Original');
ax2 = subplot(1,2,2);

% Getting image size;
size = net.Layers(1).InputSize;

% Infinite cyclenet
while ishandle(h)
    % Getting webcam image, resizing and labeling
    img = snapshot(cam);
    imgR = imresize(img, size(1, 1:2));
    [label,score] = classify(net, imgR);

    %Select the top 5 predictions after the classes with the highest scores.
    [~,idx] = sort(score,'descend');
    idx = idx(5:-1:1);
    
    classes = net.Layers(end).Classes;
    
    classNamesTop = string(classes(idx));
    
    scoreTop = score(idx);

    %Display the top five predictions as a histogram.
    barh(ax2,scoreTop)
    xlim(ax2,[0 1])
    
    title(ax2,'Top 5')
    xlabel(ax2,'Probability')
    yticklabels(ax2,classNamesTop)
    
    ax2.YAxisLocation = 'right';

    % Visualization 
    ims.CData = img;
    tt.String = label;
end
clear("cam")

%% Exercise 4
clear, clc, close all

% Calling the neural network
net = googlenet;
net2 = alexnet;

% Handle to read camera
cam = webcam(1);

% Show webcam image
img = snapshot(cam);
h = figure(1);
h.Position(3) = 3*h.Position(3);
ax1 = subplot(1,3,1);
ims = imshow(img);
tt = title('Original');
ax2 = subplot(1,3,2);
ax3 = subplot(1,3,3);

% Getting image size;
size = net.Layers(1).InputSize;

% Infinite cyclenet
while ishandle(h)
    % Getting webcam image, resizing and labeling
    img = snapshot(cam);
    imgR = imresize(img, size(1, 1:2));
    imgR2 = imresize(img, [227, 227]);
    [label, score] = classify(net, imgR);
    [label2, score2] = classify(net2, imgR2);
    
    % GoogleNet
    %Select the top 5 predictions after the classes with the highest scores.
    [~,idx] = sort(score,'descend');
    idx = idx(5:-1:1);
    
    classes = net.Layers(end).Classes;
    
    classNamesTop = string(classes(idx));
    
    scoreTop = score(idx);

    %Display the top five predictions as a histogram.
    barh(ax2,scoreTop)
    xlim(ax2,[0 1])
    
    title(ax2,'Top 5')
    xlabel(ax2,'Probability')
    yticklabels(ax2,classNamesTop)
    
    ax2.YAxisLocation = 'right';

     % AlexNet
    %Select the top 5 predictions after the classes with the highest scores.
    [~,idx2] = sort(score2,'descend');
    idx2 = idx2(5:-1:1);
    
    classes2 = net2.Layers(end).Classes;
    
    classNamesTop2 = string(classes2(idx2));
    
    scoreTop2 = score2(idx2);

    %Display the top five predictions as a histogram.
    barh(ax3,scoreTop2)
    xlim(ax3,[0 1])
    
    title(ax3,'Top 5')
    xlabel(ax3,'Probability')
    yticklabels(ax3,classNamesTop2)
    
    ax3.YAxisLocation = 'right';

    % Visualization 
    ims.CData = img;
    tt.String = label;
end
clear("cam")
