clc
clear all
close all

addpath(genpath(cd))

%% Probabilistic Roadmap Path Planning
% resource: https://www.youtube.com/watch?v=rLoSCQbiKvE

%% Step 1: Load Map and Inflate It
image        = imread('map1.png');
bwimage      = image < 0.5;
map         = binaryOccupancyMap(bwimage);
inflatedMap = copy(map);
inflate(inflatedMap,2);
aksMap = subplot(1,2,1); show(map);
aksInflatedMap = subplot(1,2,2); show(inflatedMap);

%% Step 2: Generate Roadmaps with Initial Parameters
prm = mobileRobotPRM;
prm.NumNodes = 1;
prm.ConnectionDistance = 5;
prm.Map = inflatedMap;
show(prm)

%% Step 3: Query the PRM Method
disp('Click for the Start Location')
startLocation = ginput(1);
disp('Click for the Goal Location')
endLocation = ginput(1);

path = findpath(prm,startLocation,endLocation);

while isempty(path)
    prm.NumNodes = prm.NumNodes + 20;
    update(prm)
    path = findpath(prm,startLocation,endLocation);
    show(prm)
    pause(0.1)
end
