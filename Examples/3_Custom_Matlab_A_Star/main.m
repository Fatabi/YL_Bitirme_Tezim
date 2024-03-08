clc
clear all
close all
addpath(genpath(cd))

%% Load Map
image     = imread('Map_Big.png');
bwimage   = image < 0.5;    
map       = binaryOccupancyMap(bwimage);
fig = figure;
fig.Color = [1,1,1];
aks = show(map);
hold on;
grid minor;

%% Select Start and End Point
disp("Select start point.")
startPoint = round(ginput(1));
scatter(startPoint(1),startPoint(2),200,'xg')

disp("Select end point.")
endPoint = round(ginput(1));
scatter(endPoint(1),endPoint(2),200,'xr')

%% Create Objects for A*
map = Map(startPoint,endPoint,map);

%% Solve for A Star
a_star = A_Star(map);
a_star.Solve();







