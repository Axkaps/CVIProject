clc; clear all; close all;

pathToImages = '.\Crowd_PETS\S2\L1\Time_12-34\View_001\';
frameIdComp = 4;
str = ['%sframe_%0' num2str(frameIdComp) 'd.%s']; extName = 'jpg';
numFrames = 794; % nFrames = 795

pathToFile = '.\PETS-S2L1\gt\gt.txt';
%pathToFile = '../PETS-S2L1/gt/gt.txt';
groundTruth = csvread(pathToFile);

drawTrajectory = false;

sectionInput = input("What project section do you want to run? ");

switch sectionInput
    case 1 %Only plot GT
        run('readGroundTruth.m')
    case 2 %Detect pedestrian, buth GT and ours without trajectory
        drawTrajectory = false;
        run('calculateBackground.m')
        run('detectPedestrian.m')
    case 3 %Detect pedestrian, buth GT and ours with trajectory
        drawTrajectory = true;
        run('calculateBackground.m')
        run('detectPedestrian.m')
    otherwise
        disp('Invalid section')
end

% Now lets get the GroundTruth
%run("readGroundTruth.m");

% Now we get the bounding boxes using our algorithm
%run("detectPedestrian.m");