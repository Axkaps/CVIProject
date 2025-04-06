clc; clear; close all; 

pathToImages = '.\Crowd_PETS\S2\L1\Time_12-34\View_001\';
%pathToImages = '../Crowd_PETS/S2/L1/Time_12-34/View_001/';
frameIdComp = 4;
str = ['%sframe_%0' num2str(frameIdComp) 'd.%s']; extName = 'jpg';
numFrames = 795;

pathToFile = '.\PETS-S2L1\gt\gt.txt';
%pathToFile = '../PETS-S2L1/gt/gt.txt';
groundTruth = csvread(pathToFile);

drawTrajectory = false;
drawHeatmap = false;
evaluatePerformance = false;

skeleton = struct(...
    'ID', [], ...
    'Centroid', [], ...
    'BoundingBox', [], ...
    'Trajectory', [], ...
    'Histogram', {{}}, ... % There might be more than one guy
    'LastSeen', 0 ...
);

% Pre-allocate an array of 19 structs
pedestrianDB = repmat(skeleton, 1, 20);
disp("Calculating database and background...");
[vid4D, bkg] = calculateBackground(numFrames, str, pathToImages, extName);
pedestrianDB = buildDB(pedestrianDB, numFrames, groundTruth, str, pathToImages, extName);

fprintf(['\n' ...
    , ...
         'Usage of the program:\n', ...
         '1 - Plot groundTruth\n', ...
         '2 - Detect pedestrians\n', ...
         '3 - Detect pedestrians with trajectory draw\n', ...
         '5 - Draw HeatMaps\n' ...
         '7 - Provide an evaluation performance\n']);



while true
    sectionInput = input('What project section do you want to run? (1, 2, 3, 5, 7, or 0 to exit): ');

    switch sectionInput
        case 1 % Only plot GT
            run('readGroundTruth.m')
        case 2 % Detect pedestrian, both GT and ours without trajectory
            run('detectPedestrian.m')
        case 3 % Detect pedestrian, both GT and ours with trajectory
            drawTrajectory = true;
            run('detectPedestrian.m')
        case 5 % Draw Heatmaps
            drawHeatmap = true;
            run('detectPedestrian.m')
        case 7 % Evaluate performance
            evaluatePerformance = true;
            run('detectPedestrian.m')
        case 8
            run("DLYOLO.m")
        case 0 % Exit
            disp('Exiting program. Goodbye!')
            break
        otherwise
            disp('Invalid section. Please try again.')
    end
end