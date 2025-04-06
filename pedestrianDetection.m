function [pedestrianDb, currentID, next_id] = pedestrianDetection(pedestrianDb, imgToProcess, regionProps, j, next_id, assigned, associationMatrix, regionIndex)

        
        maxMatchScore = -inf;  % Reset for each detection
        best_match_idx = -1;
        max_trajec_len = 10;
        centroid = regionProps(j).Centroid;
        bbox = regionProps(j).BoundingBox;
        currentID = [];
        intersectionThreshold = 0.5;

        similarityThreshold = 1.2; 
        distanceThreshold = 100;
        

         % Before processing, check for pedestrians not seen for more than 5 frames
        maxFramesAbsent = 5;
        for k = 1:length(pedestrianDb)
            if ~isempty(pedestrianDb(k).ID) && ~ismember(pedestrianDb(k).ID, assigned) && exist('frameNumber', 'var') && (frameNumber - pedestrianDb(k).last_seen) > maxFramesAbsent
                % Reset pedestrian if not seen for more than 5 frames
                pedestrianDb(k).ID = pedestrianDb(k).ID;
                pedestrianDb(k).Centroid = [];
                pedestrianDb(k).BoundingBox = [];
                pedestrianDb(k).Trajectory = [];
                pedestrianDb(k).Histogram = pedestrianDb(k).Histogram;
                pedestrianDb(k).last_seen = 0;
            end
        end

        % Check for merge situation using the association matrix
        if exist('associationMatrix', 'var') && ~isempty(associationMatrix) && exist('regionIndex', 'var')
            detectionCol = sum(associationMatrix(:, regionIndex) > 0);
            
            if detectionCol > 1
                currentID = -1;
                return;
            end
        end

        % Check for intersection with bounding boxes from the last frame
        for k = 1:length(pedestrianDb)
            % Skip if this ID is already assigned in this frame
            if ismember(pedestrianDb(k).ID, assigned) | isempty(pedestrianDb(k).BoundingBox)
                continue;
            end
            
            % Compute IoU with this pedestrian's bounding box
            IoU = computeIoU(bbox, pedestrianDb(k).BoundingBox);
            
            % If strong intersection, assign the same ID without further processing
            if IoU > intersectionThreshold
                currentID = pedestrianDb(k).ID;
                pedestrianDb(k).Centroid = centroid;
                pedestrianDb(k).BoundingBox = bbox;
                if exist('frameNumber', 'var')
                    pedestrianDb(k).last_seen = frameNumber; % Update last seen frame
                end
                % Update trajectory
                pedestrianDb(k).Trajectory = [pedestrianDb(k).Trajectory; centroid];
                if size(pedestrianDb(k).Trajectory, 1) > max_trajec_len
                    pedestrianDb(k).Trajectory(1, :) = []; % Remove oldest point
                end
                
                return; % Exit early since we found a match based on intersection
            end
        end
        
        croppedImage = imcrop(imgToProcess, bbox);
        R = histcounts(croppedImage(:,:,1), 256);
        G = histcounts(croppedImage(:,:,2), 256);
        B = histcounts(croppedImage(:,:,3), 256);
        histCroppedImage = [R,G,B];

        
        for k = 1:length(pedestrianDb)
            % Skip if this ID is already assigned in this frame
            if ismember(pedestrianDb(k).ID, assigned)
                continue;
            end

             % Handle empty centroid case as you requested
            if isempty(pedestrianDb(k).Centroid)
                centroidDistNorm = 0;
                centroidDist = distanceThreshold - 0.1; % Set to threshold so it passes the check below
            else
                centroidDist = norm(centroid - pedestrianDb(k).Centroid);
                centroidDistNorm = exp(-centroidDist / distanceThreshold);
            end
            
            % Try to match using histogram and centroid distance
            for i = 1:length(pedestrianDb(k).Histogram)
                bDistance = histogramDistance(pedestrianDb(k).Histogram{i}, histCroppedImage);
                histDistNorm = exp(-bDistance / similarityThreshold);
                
                if bDistance < similarityThreshold && centroidDist < distanceThreshold
                    % Compute final match score
                    matchScore = 1.2 * histDistNorm + 0.2 * centroidDistNorm;

                    if matchScore > maxMatchScore
                        maxMatchScore = matchScore;
                        best_match_idx = k;
                    end
                end
            end
        end
        
        % Either update existing pedestrian or create a new one
        if best_match_idx ~= -1
            currentID = pedestrianDb(best_match_idx).ID;
            pedestrianDb(best_match_idx).Centroid = centroid;
            pedestrianDb(best_match_idx).BoundingBox = bbox;
            if exist('frameNumber', 'var')
                pedestrianDb(best_match_idx).last_seen = frameNumber; % Update last seen frame
            end
            pedestrianDb(best_match_idx).Trajectory = [pedestrianDb(best_match_idx).Trajectory; centroid];
            if size(pedestrianDb(best_match_idx).Trajectory, 1) > max_trajec_len
                pedestrianDb(best_match_idx).Trajectory(1, :) = []; % Remove oldest point
            end
        else
            currentID = -1;
        end
end