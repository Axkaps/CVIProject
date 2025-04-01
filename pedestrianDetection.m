function [pedestrianDb, currentID, next_id] = pedestrianDetection(pedestrianDb, imgToProcess, regionProps, j, next_id, assigned, associationMatrix, regionIndex)
        % regionIndex is the index of this region in the filtered list (1 to regnum)
        
        maxMatchScore = -inf;  % Reset for each detection
        best_match_idx = -1;
        max_trajec_len = 10;
        padding = 10;
        centroid = regionProps(j).Centroid;
        bbox = regionProps(j).BoundingBox;
        currentID = [];
        intersectionThreshold = 0.5;
        
        % Check for merge situation using the association matrix
        if exist('associationMatrix', 'var') && ~isempty(associationMatrix) && exist('regionIndex', 'var')
            % Count how many ground truth objects are associated with this detection
            % Use regionIndex instead of j to access the correct column
            detectionCol = sum(associationMatrix(:, regionIndex) > 0);
            
            % If more than one pedestrian is inside this bounding box (merge situation)
            if detectionCol > 1
                % In case of merge, don't process further
                currentID = -1; % Use a special ID to indicate merge
                return;
            end
        end

        % Check for intersection with bounding boxes from the last frame
        for k = 1:length(pedestrianDb)
            % Skip if this ID is already assigned in this frame
            if ismember(pedestrianDb(k).ID, assigned)
                continue;
            end
            
            % Compute IoU with this pedestrian's bounding box
            IoU = computeIoU(bbox, pedestrianDb(k).BoundingBox);
            
            % If strong intersection, assign the same ID without further processing
            if IoU > intersectionThreshold
                currentID = pedestrianDb(k).ID;
                pedestrianDb(k).Centroid = centroid;
                pedestrianDb(k).BoundingBox = bbox;
                
                % Update trajectory
                pedestrianDb(k).Trajectory = [pedestrianDb(k).Trajectory; centroid];
                if size(pedestrianDb(k).Trajectory, 1) > max_trajec_len
                    pedestrianDb(k).Trajectory(1, :) = []; % Remove oldest point
                end
                
                return; % Exit early since we found a match based on intersection
            end
        end
        
        % If we reach here, no strong intersection was found
        % Continue with the original histogram and centroid-based matching
        
        % Crop the original image
        croppedImage = imcrop(imgToProcess, bbox);
        
        % Compute histogram for this croppedImage
        R = histcounts(croppedImage(:,:,1), 256);
        G = histcounts(croppedImage(:,:,2), 256);
        B = histcounts(croppedImage(:,:,3), 256);
        histCroppedImage = [R,G,B];

        similarityThreshold = 1.2; 
        distanceThreshold = 100;
        
        for k = 1:length(pedestrianDb)
            % Skip if this ID is already assigned in this frame
            if ismember(pedestrianDb(k).ID, assigned)
                continue;
            end
            % TODO: Fazer L2 norm
            centroidDist = norm(centroid - pedestrianDb(k).Centroid);
            centroidDistNorm = exp(-centroidDist / distanceThreshold);
            
            % Try to match using histogram and centroid distance
            for i = 1:length(pedestrianDb(k).Histogram)
                bDistance = histogramDistance(pedestrianDb(k).Histogram{i}, histCroppedImage);

                if bDistance < similarityThreshold && centroidDist < distanceThreshold
                    % Compute final match score
                    matchScore = 1/bDistance + 1/centroidDist;

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

            pedestrianDb(best_match_idx).Trajectory = [pedestrianDb(best_match_idx).Trajectory; centroid];
            if size(pedestrianDb(best_match_idx).Trajectory, 1) > max_trajec_len
                pedestrianDb(best_match_idx).Trajectory(1, :) = []; % Remove oldest point
            end
        else
            currentID = next_id;
            pedestrianDb(end + 1) = struct("ID", next_id, "Centroid", centroid, ...
                                         "BoundingBox", bbox, "Trajectory", centroid, ...
                                         "Histogram", {{histCroppedImage}});
            next_id = next_id + 1;
        end
end