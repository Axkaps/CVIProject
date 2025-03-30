function [pedestrianDb, currentID, next_id] = pedestrianDetection(pedestrianDb, imgToProcess, lb, regionProps, j, next_id)
        
        maxMatchScore = -inf;  % Reset for each detection
        best_match_idx = -1;
        max_trajec_len = 10;
        padding = 10;
        singleRegionMask = (lb == j);
        centroid = regionProps(j).Centroid;
        bbox = regionProps(j).BoundingBox;
        currentID = [];

        bbox = [
            max(1, bbox(1) - padding),
            max(1, bbox(2) - padding),
            min(bbox(3) + 2*padding, size(imgToProcess, 2) - bbox(1)), % width
            min(bbox(4) + 2*padding, size(imgToProcess, 1) - bbox(2))  % height
        ];

        % Crop the original image
        croppedImage = imcrop(imgToProcess, bbox);
        
        % (Optional) Apply the mask to the cropped region
        croppedMask = imcrop(singleRegionMask, bbox);
        maskedImage = croppedImage;
        maskedImage(repmat(~croppedMask, [1, 1, 3])) = 0; % For RGB images
        
        % Compute histogram for this maskedImage
        histMaskedImage = computeColorHistogram(maskedImage);

        best_match_idx = -1;
        intersectionTreshold = 0.5;
        similarityTreshold = 0.7; % Tem de ser menor que isto
        distanceThreshold = 100;
        maxIoU = 0;
        for k = 1:length(pedestrianDb)                    

            bDistance = bhattacharyya(pedestrianDb(k).Histogram, histMaskedImage);
            centroidDist = norm(centroid - pedestrianDb(k).Centroid);
            centroidDistNorm = exp(-centroidDist / distanceThreshold);
            IoU = computeIoU(bbox, pedestrianDb(k).BoundingBox);
            % First compute the intersection, then if the results are not
            % promising use histogram and centroid to try to Re-ID
            if IoU > intersectionTreshold && IoU > maxIoU
                maxIoU = IoU;
                best_match_idx = k;
            end


            % Weighted fusion of different similarity measures
            alpha = 0.6;  % Weight for color similarity
            beta = 0.3;   % Weight for spatial consistency

            if maxIoU < intersectionTreshold + 0.1 && bDistance < similarityTreshold && centroidDist < distanceThreshold
                % Compute final match score
                matchScore = alpha * (1 - bDistance) + beta * centroidDistNorm;
                
                if matchScore > maxMatchScore
                    maxMatchScore = matchScore;
                    best_match_idx = k;
                end
            end
        end
        

        if best_match_idx ~= -1
            currentID = pedestrianDb(best_match_idx).ID;
            pedestrianDb(best_match_idx).Centroid = centroid;
            pedestrianDb(best_match_idx).BoundingBox = bbox;
            pedestrianDb(best_match_idx).Histogram = histMaskedImage;

            pedestrianDb(best_match_idx).Trajectory = [pedestrianDb(best_match_idx).Trajectory; centroid];
            if size(pedestrianDb(best_match_idx).Trajectory, 1) > max_trajec_len
                pedestrianDb(best_match_idx).Trajectory(1, :) = []; % we need to remove oldest point in order to get the trail
            end
        else
            currentID = next_id;
            pedestrianDb(end + 1) = struct("ID", next_id, "Centroid", centroid, ...
                                         "BoundingBox", bbox, "Trajectory", centroid, ...
                                         "Histogram", histMaskedImage);
            next_id = next_id + 1;
        end
end