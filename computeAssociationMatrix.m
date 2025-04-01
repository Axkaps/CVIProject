function [successPercentage, C] = computeAssociationMatrix(groundTruthMatrix, regionProps, inds, height, width, i, regnum)
    
    DetectorMatrix = zeros(height, width);
    successPercentage = 0;

    for j=1:regnum
        
        bbox = regionProps(inds(j)).BoundingBox;
        adjusted_bbox = [bbox(1)-0.5, bbox(2)-0.5, bbox(3), bbox(4)];
        
        % Convert BoundingBox coordinates to integer indices
        x_min = floor(adjusted_bbox(1)) + 1; % Leftmost column (1-based)
        y_min = floor(adjusted_bbox(2)) + 1; % Topmost row (1-based)
        x_max = floor(adjusted_bbox(1) + adjusted_bbox(3)); % Rightmost column
        y_max = floor(adjusted_bbox(2) + adjusted_bbox(4)); % Bottommost row

        % Ensure indices are within matrix bounds
        x_min = max(1, x_min);
        y_min = max(1, y_min);
        x_max = min(width, x_max);
        y_max = min(height, y_max);

        DetectorMatrix(y_min:y_max, x_min:x_max) = 1;
        

    end
    [L_gt, num_gt] = bwlabel(groundTruthMatrix(:,:, i));    % Label GT objects
    [L_det, num_det] = bwlabel(DetectorMatrix);  
    
    C = zeros(num_gt, regnum); % Initialize association matrix

    for l = 1:num_gt
        for m = 1:regnum
            % Find pixels belonging to GT object i
            gt_mask = (L_gt == l);
    
            % Find pixels belonging to detected object j
            det_mask = (L_det == m);
    
            % Compute Intersection and Union
            intersection = sum(gt_mask(:) & det_mask(:));
            union = sum(gt_mask(:) | det_mask(:)); 
    
            % Store Intersection over Union (IoU)
            if union > 0
                C(l, m) = (intersection / union) > 0.5;
                successPercentage = successPercentage + (intersection / union);
            end
        end
    end
    successPercentage = successPercentage / (num_gt);

end