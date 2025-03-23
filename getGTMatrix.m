function groundTruthMatrix = getGTMatrix(groundTruth, numFrames)
    
    height = 576;
    width = 768;
    groundTruthMatrix = zeros(height, width, numFrames);

    
    for i = 1:numFrames
        frameData = groundTruth(groundTruth(:,1) == i, :);
        
        for k = 1:size(frameData, 1)
            ID = frameData(k, 2);
            x = round(frameData(k, 3));
            y = round(frameData(k, 4));
            w = round(frameData(k, 5));
            h = round(frameData(k, 6));

            % Ensure bounding box is within image limits
            y_end = min(y + h, height);
            x_end = min(x + w, width);
            y = max(1, y);
            x = max(1, x);

            groundTruthMatrix(y:y_end, x:x_end, i) = ID;
        end
    end
end


