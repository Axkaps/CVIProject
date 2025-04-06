function [successPercentage, associationMatrix] = computeAssociationMatrix(groundTruth, regionProps, inds, currentFrame, threshold)

    % Extract ground truth for current frame
    gtForFrame = groundTruth(groundTruth(:,1) == currentFrame, :);
    numGT = size(gtForFrame, 1);
    successPercentage = 0;

    % Get number of filtered detections
    numDetections = length(inds);
    
    % Initialize association matrix
    associationMatrix = zeros(numGT, numDetections);
    
    % Compute binary association for each ground truth - filtered detection pair
    for i = 1:numGT
        gtID = gtForFrame(i, 2);
        gtX = gtForFrame(i, 3);
        gtY = gtForFrame(i, 4);
        gtW = gtForFrame(i, 5);
        gtH = gtForFrame(i, 6);
        gtBox = [gtX, gtY, gtW, gtH];
        
        for j = 1:numDetections
            % Get the actual index in regionProps
            detIdx = inds(j);
            
            % Extract bounding box from regionprops
            detBox = regionProps(detIdx).BoundingBox;
            
            % Calculate IoU using existing function
            iou = computeIoU(gtBox, detBox);
            
            % Set binary association based on threshold
            if iou > threshold
                associationMatrix(i, j) = 1;
            end
            successPercentage = successPercentage + iou;
        end
    end
    successPercentage = successPercentage / (numGT);
end