clc; clear all; close all; 

%pathToImages = '.\Crowd_PETS\S2\L1\Time_12-34\View_001\';
pathToImages = '../Crowd_PETS/S2/L1/Time_12-34/View_001/';
frameIdComp = 4;
str = ['%sframe_%0' num2str(frameIdComp) 'd.%s']; extName = 'jpg';
numFrames = 795;

%pathToFile = '.\PETS-S2L1\gt\gt.txt';
pathToFile = '../PETS-S2L1/gt/gt.txt';
groundTruth = csvread(pathToFile);

drawTrajectory = false;
drawHeatmap = false;

disp("Calculating database and background..")
run('calculateBackground.m');
buildHistogramDB(vid4D);
fprintf(['\n', ...
         'Usage of the program:\n', ...
         '1 - Plot groundTruth\n', ...
         '2 - Detect pedestrians\n', ...
         '3 - Detect pedestrians with trajectory draw\n', ...
         '4 - Draw HeatMaps\n']);

sectionInput = input("What project section do you want to run? ");

switch sectionInput
    case 1 % Only plot GT
        run('readGroundTruth.m')
    case 2 % Detect pedestrian, buth GT and ours without trajectory
        run("getGroundTruthMatrix.m")
        
        run('detectPedestrian.m')
    case 3 % Detect pedestrian, buth GT and ours with trajectory
        drawTrajectory = true;
        run("getGroundTruthMatrix.m")
        run('detectPedestrian.m')
    case 5 % Draw Heatmaps
        drawHeatmap = true;
        run("getGroundTruthMatrix.m")
        run('detectPedestrian.m')
    otherwise
        disp('Invalid section')
end