function pedestrianDb = buildHistogramDB(vid4D)
    pedestrianDb = struct("ID", {}, "Centroid", {}, "BoundingBox", {}, "Trajectory", {}, "Histogram", {});
    
    numFrames = size(vid4D, 4);
    background = rgb2gray(vid4D(:, :, :, 1));  
    next_id = 1;

    similarityThreshold = 0.1;  
    distanceThreshold = 30;    

    for frameIdx = 2:numFrames
        currentFrame = vid4D(:, :, :, frameIdx);
        grayFrame = rgb2gray(currentFrame);
        foregroundMask = abs(double(grayFrame) - double(background)) > 30; 

        foregroundMask = imopen(foregroundMask, strel('disk', 3));
        foregroundMask = imclose(foregroundMask, strel('disk', 5));
        foregroundMask = bwareafilt(foregroundMask, [500, Inf]);

        stats = regionprops(foregroundMask, 'BoundingBox', 'Centroid');

        for j = 1:length(stats)
            bbox = stats(j).BoundingBox;
            centroid = stats(j).Centroid;
            pedestrianRegion = imcrop(currentFrame, bbox);
            histFeature = computeColorHistogram(pedestrianRegion);

            isDuplicate = false;

            for k = 1:length(pedestrianDb)
                bDistance = bhattacharyya(pedestrianDb(k).Histogram, histFeature);
                centroidDist = norm(centroid - pedestrianDb(k).Centroid);

                if bDistance < similarityThreshold || centroidDist < distanceThreshold
                    isDuplicate = true;
                    break;
                end
            end

            if ~isDuplicate
                pedestrianDb(next_id).ID = next_id;
                pedestrianDb(next_id).Centroid = centroid;
                pedestrianDb(next_id).BoundingBox = bbox;
                pedestrianDb(next_id).Trajectory = centroid;
                pedestrianDb(next_id).Histogram = histFeature;
                next_id = next_id + 1;
            end
        end
    end

    save('pedestrianDB.mat', 'pedestrianDb');
end
