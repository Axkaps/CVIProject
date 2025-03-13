clc; clear all; close all;
% Path no mac -
%pathToImages = '../Crowd_PETS/S2/L1/Time_12-34/View_001/';
pathToImages = '.\Crowd_PETS\S2\L1\Time_12-34\View_001\';
frameIdComp = 4;
str = ['%sframe_%0' num2str(frameIdComp) 'd.%s']; extName = 'jpg';
numFrames = 794; % nFrames = 795
step = 1; i = 1;

figure; hold on,
for k =0:step:numFrames
    k
    img= imread(sprintf(str,pathToImages,k,extName));   
    
    vid4D(:,:,:,i) = img;
    imshow(img); drawnow
    i = i+1;
end

% Get the background (Should try both median and low pass i guess)
bkg = median(vid4D,4); 
figure; hold on,
imshow(bkg);

% Lets define the strucutring element from now on
nPedestrians = 10;
pedestrianDb(nPedestrians) = struct("ID", 0, "Centroid", [0, 0]);

for i = 1:10
    pedestrianDb(i).ID = i;
    pedestrianDb(i).Centroid = [i, i];
end

% Now lets get the GroundTruth
run("readGroundTruth.m");

% Now we get the bounding boxes using our algorithm
run("detectPedestrian.m");