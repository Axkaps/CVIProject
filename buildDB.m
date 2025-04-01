function pedestrianDB = buildDB(pedestrianDB, numFrames, groundTruth, str, pathToImages, extName)
    
    % Loop through each detection in the frame and construct the DB from GT
    for i = 0:numFrames - 1
        pedestrianDB = buildHistogramDB(pedestrianDB, i, groundTruth, str, pathToImages, extName);
    end

    % Trying to add expressiveness to the algorithm
    for l=1:length(pedestrianDB)
        meanHist = (pedestrianDB(l).Histogram{1} + pedestrianDB(l).Histogram{2}) / 2;
        pedestrianDB(l).Histogram{end+1} = meanHist; 
    end

end