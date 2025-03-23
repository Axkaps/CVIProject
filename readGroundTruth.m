figure; hold on;

% Loop through each detection in the frame and plot the bounding box
for i = 0:numFrames
    drawGT(i, groundTruth, str, pathToImages, extName);
end

