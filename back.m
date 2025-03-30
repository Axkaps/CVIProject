    numFrames = 395;
    background = rgb2gray(bkg);  
    next_id = 1;
    minArea = 200;  
    maxArea = 2500;  
    similarityThreshold = 0.5;  
    distanceThreshold = 100;    
    iouThreshold = 0.2;

    for frameIdx = 195:numFrames
        currentFrame = vid4D(:, :, :, frameIdx);
        grayFrame = rgb2gray(currentFrame);
        foregroundMask = abs(double(grayFrame) - double(background)) > 30; 

        foregroundMask = imopen(foregroundMask, strel('disk', 2));
        foregroundMask = imclose(foregroundMask, strel('disk', 5));
        foregroundMask = bwareafilt(foregroundMask, [100, Inf]);
        
        stats = regionprops(foregroundMask, 'Area', 'BoundingBox', 'Centroid');
        inds = find([stats.Area] > minArea & [stats.Area] < maxArea);

        for j = inds
            bbox = stats(j).BoundingBox;
            centroid = stats(j).Centroid;
            pedestrianRegion = imcrop(currentFrame, bbox);
            histFeature = computeColorHistogram(pedestrianRegion);

            isDuplicate = false;

            for k = 1:length(pedestrianDb)
                bDistance = bhattacharyya(pedestrianDb(k).Histogram, histFeature);
                centroidDist = norm(centroid - pedestrianDb(k).Centroid);
                iouScore = computeIoU(bbox, pedestrianDb(k).BoundingBox);

                if bDistance < similarityThreshold && centroidDist < distanceThreshold 
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