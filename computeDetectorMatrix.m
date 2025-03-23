function DetectorMatrix = computeDetectorMatrix(regnum, inds, regionProps)
height = 576;
width = 768;
DetectorMatrix = zeros(height, width);

    for j=1:regnum
    
            bbox = regionProps(inds(j)).BoundingBox;
            % Basta criar uma frame para comparar e fazer o split nao
            % preciso de guardar para nada
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
end