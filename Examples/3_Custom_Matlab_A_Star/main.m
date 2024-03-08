clc
clear
close all
addpath(genpath(cd))

%% User Input
showMode = false;

%% Load Map
image     = imread('Map_Big.png');
bwimage   = image < 0.5;
map       = binaryOccupancyMap(bwimage);
fig = figure;
fig.Color = [1,1,1];
fig.Position = [1 41 1920 963];
aks = show(map);
hold on;
grid minor;

%% Select Start and End Point
if showMode == true
    disp("Select start point.")
    startPoint = round(ginput(1));
    
    disp("Select end point.")
    endPoint = round(ginput(1));
else
    % startPoint = [70,60];
    % endPoint   = [20,80];
    
    % startPoint = [40,60];
    % endPoint   = [60,90];
    
    % startPoint = [10,80];
    % endPoint   = [10,10];

    startPoint = [80,20];
    endPoint   = [80,60];
end
scatter(startPoint(1),startPoint(2),200,'xg')
scatter(endPoint(1),endPoint(2),200,'xr')
pathLine = scatter(startPoint(1),startPoint(2),'DisplayName','Path');

%% Create Objects for A*
map = Map(startPoint,endPoint,map);

%% Solve for A Star
a_star = A_Star(map,'drawNow',true,'pathLine',pathLine);
a_star.Solve();

%% Draw Path
pathLoc = a_star.CreatePathLocation();
plot(pathLoc(:,1),pathLoc(:,2),'DisplayName','Path','LineWidth',3)