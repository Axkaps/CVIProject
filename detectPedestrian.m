thr = 50;
minArea = 200;
maxArea = 7000;
ID = 1;
se = strel('rectangle', [3 10]);
max_trajec_len = 10;
next_id = 1;
pedestrianDb = struct("ID", {}, "Centroid", {}, "BoundingBox", {}, "Trajectory", {}); 

%maxpeople, maxframes
%centroids = zeros(15, 20); 
%trajectoryFrame = 0;

figure; hold on
for i=1:size(vid4D, 4)

    imgfr = vid4D(:,:,:,i);
    imgdif = (abs(double(bkg(:,:,1))-double(imgfr(:,:,1))) > thr) | ...
        (abs(double(bkg(:,:,2))-double(imgfr(:,:,2))) > thr) | ...
        (abs(double(bkg(:,:,3))-double(imgfr(:,:,3))) > thr);

    %Draw GT bbox (not working)
    frameData = groundTruth(groundTruth(:,1) == (i + 1), :);
    imagePath = sprintf(str,pathToImages,i,extName);

    cla;
    % Display the image

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


    

    % bw1 = imclose(imgdif,se);
    % bw2 = imerode(bw1,se);
    bw = imgdif;
    % bw = imerode(bw, se); % Reduce objects trying to mitigate double detection
    % bw = imdilate(bw, se); % tentativa
   
    [lb num]=bwlabel(bw);
    regionProps = regionprops(lb, 'Area', 'BoundingBox', 'Centroid');
    inds = find([regionProps.Area] > minArea & [regionProps.Area] < maxArea);
    
    regnum = length(inds);

    clf;
    imshow(imgfr);
    hold on;

    if regnum
        for j=1:regnum
            [lin col]= find(lb == inds(j));
            upLPoint = min([lin col]);
            dWindow  = max([lin col]) - upLPoint + 1;
            
            %rectangle('Position',[fliplr(upLPoint) fliplr(dWindow)],'EdgeColor',[1 1 0],...
             %   'linewidth',2);

            %trajectoryFrame = trajectoryFrame + 1; 
            %if trajectoryFrame == 20
                %trajectoryFrame = 1;
            %end

            %delete(centroids(:, trajectoryFrame));

            centroid = regionProps(inds(j)).Centroid;
            bbox = regionProps(inds(j)).BoundingBox;

            assigned_id = -1;
            max_iou = 0;
            best_match_idx = -1;

            for k = 1:length(pedestrianDb)
                iou = computeIoU(bbox, pedestrianDb(k).BoundingBox);
                if iou > max_iou && iou > 0.5  % IoU threshold (like discussed during the labs)
                    max_iou = iou;
                    best_match_idx = k;
                end
            end

            if best_match_idx ~= -1
                assigned_id = pedestrianDb(best_match_idx).ID;
                pedestrianDb(best_match_idx).Centroid = centroid;
                pedestrianDb(best_match_idx).BoundingBox = bbox;

                pedestrianDb(best_match_idx).Trajectory = [pedestrianDb(best_match_idx).Trajectory; centroid];
                if size(pedestrianDb(best_match_idx).Trajectory, 1) > max_trajec_len
                    pedestrianDb(best_match_idx).Trajectory(1, :) = []; % we need to remove oldest point in order to get the trail
                end
            else
                assigned_id = next_id;
                pedestrianDb(end+1) = struct("ID", next_id, "Centroid", centroid, ...
                                             "BoundingBox", bbox, "Trajectory", centroid);
                next_id = next_id + 1;
            end

            if drawTrajectory
                plot(centroid(1), centroid(2), 'g.', 'MarkerSize', 20);

                if best_match_idx ~= -1
                    traj = pedestrianDb(best_match_idx).Trajectory;
                    if size(traj, 1) > 1
                        plot(traj(:, 1), traj(:, 2), 'g-', 'LineWidth', 2); 
    
                    end
                end
            end
            
            %centroids(j, trajectoryFrame) = r;
            
            textPosition = [fliplr(upLPoint)  - [0, 10]];
            % Display pedestrian ID
            text(textPosition(1), textPosition(2), sprintf('ID: %d', ID), 'Color', 'yellow', ...
            'FontSize', 10, 'FontWeight', 'bold');
            ID = ID + 1;
        end
        ID = 1; % Provisory labels
    end

    drawnow;
end

function iou = computeIoU(boxA, boxB)
    xA = max(boxA(1), boxB(1)); % leftmost edge of inter area
    yA = max(boxA(2), boxB(2)); % biggest y value (since we'll add the other box's height to its y value)
    xB = min(boxA(1) + boxA(3), boxB(1) + boxB(3)); % minimum since we want to know where's the rightmost edge of the inter area
    yB = min(boxA(2) + boxA(4), boxB(2) + boxB(4)); % we later get the height of the inter area

    interArea = max(0, xB - xA) * max(0, yB - yA); % cant be negative, which could happen if there was no overlap
    boxAArea = boxA(3) * boxA(4);
    boxBArea = boxB(3) * boxB(4);
    
    iou = interArea / (boxAArea + boxBArea - interArea);
end