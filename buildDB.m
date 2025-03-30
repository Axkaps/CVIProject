function pedestrianDB = buildDB(pedestrianDB, numFrames, groundTruth, str, pathToImages, extName)
    
    % Loop through each detection in the frame and construct the DB from GT
    for i = 0:numFrames - 1
        pedestrianDB = buildHistogramDB(pedestrianDB, i, groundTruth, str, pathToImages, extName);
    end

end