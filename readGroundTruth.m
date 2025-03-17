figure; hold on;

% Loop through each detection in the frame and plot the bounding box
for i = 0:numFrames

    frameData = groundTruth(groundTruth(:,1) == (i + 1), :);
    imagePath = sprintf(str,pathToImages,i,extName);
    img = imread(imagePath);

    cla;
    % Display the image
    imshow(img);

    for k = 1:size(frameData, 1)
        % Box data
        x = frameData(k, 3); % Box left
        y = frameData(k, 4); % box top
        w = frameData(k, 5); % box width
        h = frameData(k, 6); % box height
        ID = frameData(k, 2); % pedestrian ID
    
        
        % Draw bounding box
        rectangle('Position', [x, y, w, h], 'EdgeColor', 'r', 'LineWidth', 2);
    
        % Display pedestrian ID
        text(x, y - 5, sprintf('ID: %d', ID), 'Color', 'yellow', ...
            'FontSize', 10, 'FontWeight', 'bold');
    end

    drawnow;

end

