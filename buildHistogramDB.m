function pedestrianDB = buildHistogramDB(pedestrianDB, i, groundTruth, str, pathToImages, extName)

    histogramThreshold = 0.5;
    frameData = groundTruth(groundTruth(:,1) == (i + 1), :);
    imagePath = sprintf(str,pathToImages,i,extName);
    img = imread(imagePath);
    
    for k = 1:size(frameData, 1)
        % Box data
        ID = frameData(k, 2); % pedestrian ID
        x = frameData(k, 3); % Box left
        y = frameData(k, 4); % box top
        w = frameData(k, 5); % box width
        h = frameData(k, 6); % box height
        
        bbox = [x, y, w, h];

        croppedImage = imcrop(img, bbox);

        % Compute histogram for this croppedImage
        R = histcounts(croppedImage(:,:,1), 256);
        G = histcounts(croppedImage(:,:,2), 256);
        B = histcounts(croppedImage(:,:,3), 256);
        histPedestrianImage = [R,G,B];

        % If pedestrian ID doesn't exist yet, create it with first histogram
        if ID > numel(pedestrianDB) || isempty(pedestrianDB(ID).ID)
            pedestrianDB(ID).ID = ID;
            pedestrianDB(ID).Centroid = 0;
            pedestrianDB(ID).BoundingBox = bbox;
            pedestrianDB(ID).Trajectory = [];
            pedestrianDB(ID).Histogram = {histPedestrianImage};
            continue;
        end
        
        % If we have less than 4 histograms, just check if it's different enough from existing ones
        if numel(pedestrianDB(ID).Histogram) < 4
            % Compare with existing histograms
            addHistogram = true;
            
            for h = 1:numel(pedestrianDB(ID).Histogram)
                existingHist = pedestrianDB(ID).Histogram{h};
                distance = histogramDistance(existingHist, histPedestrianImage);
                
                if distance <= histogramThreshold
                    addHistogram = false;
                    break;
                end
            end
            
            if addHistogram
                pedestrianDB(ID).Histogram{end+1} = histPedestrianImage;
            end
        else
            % We already have 4 or more histograms
            % Find the closest histogram to the new one
            distances = zeros(1, numel(pedestrianDB(ID).Histogram));
            
            for h = 1:numel(pedestrianDB(ID).Histogram)
                existingHist = pedestrianDB(ID).Histogram{h};
                distances(h) = histogramDistance(existingHist, histPedestrianImage);
            end
            
            [minDist, minIdx] = min(distances);
            
            % If the new histogram is significantly different from all existing ones
            if minDist > histogramThreshold
                % Find the two most similar histograms in our collection
                pairwiseDistances = zeros(numel(pedestrianDB(ID).Histogram));
                for i1 = 1:numel(pedestrianDB(ID).Histogram)
                    for i2 = (i1+1):numel(pedestrianDB(ID).Histogram)
                        pairwiseDistances(i1, i2) = histogramDistance(pedestrianDB(ID).Histogram{i1}, pedestrianDB(ID).Histogram{i2});
                    end
                end
                
                [~, minIndices] = min(pairwiseDistances(:));
                [row, col] = ind2sub(size(pairwiseDistances), minIndices);
                
                % Replace the one that's most similar to others
                if numel(pedestrianDB(ID).Histogram) >= 5
                    pedestrianDB(ID).Histogram{row} = histPedestrianImage;
                else
                    pedestrianDB(ID).Histogram{end+1} = histPedestrianImage;
                end
            end
        end
    end
    
   % Init the woman that is not in the gt
    pedestrianDB(20).ID = 20;
    pedestrianDB(20).Centroid = 0;
    pedestrianDB(20).BoundingBox = [273.500000000000,53.5000000000000,14,31];
    pedestrianDB(20).Trajectory = [];
    pedestrianDB(20).Histogram{1} = [1	0	1	3	1	4	3	7	3	4	3	6	3	8	3	4	3	3	2	6	2	4	2	0	3	3	2	1	2	4	6	1	3	4	2	2	1	2	1	3	4	1	1	1	4	2	0	3	1	1	1	2	1	2	3	1	2	3	2	1	4	2	5	3	0	1	1	0	1	2	3	2	0	1	3	5	1	1	2	0	1	0	3	3	1	1	5	0	2	3	1	3	1	1	2	5	3	2	3	2	1	4	2	3	4	3	2	2	5	2	13	6	5	3	9	8	8	7	3	12	13	11	8	11	5	3	3	6	6	4	3	2	1	3	1	3	3	3	1	2	0	0	0	0	1	1	0	0	1	0	0	2	2	0	1	0	2	1	0	0	0	0	5	1	0	0	0	0	0	0	0	0	1	1	1	1	1	1	0	0	0	1	1	0	1	0	0	0	0	0	0	0	2	0	0	0	1	1	0	0	0	0	1	0	0	2	1	1	0	0	0	1	1	0	0	1	0	0	0	0	0	0	1	0	0	0	0	0	0	1	2	0	1	0	1	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	1	1	1	4	3	10	6	6	5	9	7	9	3	3	6	2	7	3	2	1	2	2	2	2	1	3	1	3	1	3	2	3	1	2	0	3	4	2	1	1	3	0	2	2	0	2	0	2	5	1	2	1	2	2	2	4	2	4	2	2	1	2	1	1	2	1	2	2	1	3	1	0	2	3	3	2	2	4	1	2	0	1	1	2	1	1	3	2	2	0	4	1	2	2	3	4	0	5	0	1	1	2	0	2	1	2	1	1	2	1	0	2	1	1	2	0	4	0	3	2	8	5	9	9	6	2	8	4	4	4	8	6	10	7	4	7	14	12	13	7	3	4	4	2	2	1	1	2	2	1	0	0	1	2	0	0	1	1	1	1	1	1	0	1	2	0	0	0	1	1	1	1	0	0	0	2	0	0	0	2	0	0	0	0	0	0	1	1	1	1	1	0	0	0	1	0	0	0	1	0	1	0	0	0	1	0	0	0	0	1	1	1	0	2	0	0	0	0	0	1	1	1	0	0	0	0	0	1	0	0	1	0	0	0	0	0	0	0	0	0	2	1	0	1	0	1	0	0	0	0	0	0	0	0	1	0	0	1	2	2	4	52	3	7	4	1	0	3	6	4	1	3	0	1	4	3	2	0	2	4	4	2	4	0	3	2	3	3	0	0	2	3	3	1	0	1	0	4	2	5	0	1	2	4	1	0	2	2	2	3	3	0	2	0	0	1	0	1	1	3	1	4	0	3	1	1	1	3	0	4	3	4	2	0	5	2	2	4	2	0	0	8	7	4	0	5	7	6	9	10	0	11	14	12	9	0	12	9	15	1	6	0	6	5	9	1	7	0	0	3	0	3	0	2	3	2	3	0	0	0	2	3	1	0	0	2	0	2	0	0	0	0	1	1	2	0	0	2	0	1	0	0	3	1	0	2	0	0	1	2	0	0	0	2	1	0	1	0	0	0	3	1	0	0	0	1	0	2	0	0	1	0	0	1	0	2	0	0	0	0	0	1	0	0	1	0	2	0	2	0	0	1	0	1	0	2	0	0	2	0	0	0	0	1	0	0	0	0	1	1	0	0	1	0	0	1	0	0	0	1	1	0	0	0	0	1	0	0	1	2	0	1	1	0	0	0	0	0	0	1	0	0	1	1	1	1	0	1	0	0	0	2	0	1	0	1	3];
    pedestrianDB(20).Histogram{2} = [4	14	11	8	8	6	7	9	5	6	7	10	7	11	6	8	5	7	5	1	10	3	3	0	2	2	2	3	2	1	4	0	2	4	3	4	2	2	1	2	2	0	2	2	4	0	3	0	1	2	1	0	3	0	1	1	2	1	1	0	1	1	0	0	2	0	0	1	1	0	2	0	0	1	1	1	2	1	1	0	4	1	0	0	2	2	2	2	4	1	0	2	3	2	0	0	5	1	4	6	4	4	7	4	5	5	6	6	6	7	8	3	7	4	5	5	4	8	4	0	5	7	4	2	4	5	5	4	0	4	5	3	1	0	2	1	0	0	1	1	0	1	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	1	1	0	0	0	0	1	0	1	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	1	0	0	0	0	0	0	0	0	0	1	0	1	1	0	0	0	0	0	0	1	0	0	1	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	1	0	0	1	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	2	0	1	0	1	49	19	17	15	12	6	7	4	8	6	6	1	2	2	0	4	5	2	4	4	2	4	1	1	3	6	1	2	2	5	0	1	0	3	2	2	0	3	1	0	1	1	0	0	2	1	0	1	0	1	1	0	1	1	4	1	0	1	0	1	1	1	1	1	1	1	0	0	2	0	1	3	1	0	1	2	1	2	2	0	2	1	1	2	1	1	1	2	1	2	1	1	2	1	1	2	0	0	0	2	3	4	6	4	4	8	10	6	7	8	5	8	5	8	3	8	13	6	5	10	12	4	5	3	2	3	0	1	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	1	0	0	0	0	0	1	1	0	0	1	0	0	0	0	0	1	0	1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	1	1	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1	1	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	1	0	0	0	1	0	0	0	1	0	0	0	0	0	0	0	0	1	0	0	1	116	17	7	7	6	0	6	6	4	3	0	2	1	3	7	1	0	2	0	4	0	0	1	3	4	1	0	2	2	1	2	2	0	0	1	1	4	0	3	1	3	0	0	0	1	2	0	1	0	2	1	0	2	0	0	0	2	2	0	2	3	3	4	5	0	3	4	3	5	0	4	7	7	6	0	3	3	7	5	3	0	6	6	8	5	0	3	9	2	6	0	6	10	4	8	5	0	4	2	3	2	0	2	0	1	1	0	1	2	2	1	1	0	2	2	0	0	0	4	2	2	0	0	2	2	1	0	1	0	0	1	1	0	0	2	1	0	0	1	0	0	0	0	0	0	1	0	0	0	0	2	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	1	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	1	1	0	0	0	0	2	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	1	0	1	1];
    

end