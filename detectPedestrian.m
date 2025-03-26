thr = 50;
minArea = 200;
maxArea = 7000;
se = strel('rectangle', [3 10]);
max_trajec_len = 10;
next_id = 1;
pedestrianDb = struct("ID", {}, "Centroid", {}, "BoundingBox", {}, "Trajectory", {}, "Histogram", {}); 
height = 576;
width = 768;
sigma = 20;
heatmap = zeros(height, width);
dynamicHeatmap = zeros(height, width);
heatmapDecay = 0.99;

%maxpeople, maxframes
%centroids = zeros(15, 20); 
%trajectoryFrame = 0;

% figure; hold on
for i=1:size(vid4D, 4)

    imgfr = vid4D(:,:,:,i);
    imgdif = (abs(double(bkg(:,:,1))-double(imgfr(:,:,1))) > thr) | ...
        (abs(double(bkg(:,:,2))-double(imgfr(:,:,2))) > thr) | ...
        (abs(double(bkg(:,:,3))-double(imgfr(:,:,3))) > thr);

    % bw1 = imclose(imgdif,se);
    % bw2 = imerode(bw1,se);
    bw = imgdif;
    % bw = imerode(bw, se); % Reduce objects trying to mitigate double detection
    % bw = imdilate(bw, se); % tentativa
   
    % Use lb matrix to be my detector matrix
    [lb num]=bwlabel(bw); 
    regionProps = regionprops(lb, 'Area', 'BoundingBox', 'Centroid');
    inds = find([regionProps.Area] > minArea & [regionProps.Area] < maxArea); % gotta do a new one for the split version
    
    regnum = length(inds);

    clf;
    if drawHeatmap
        subplot(2, 2, 1);
        title('Pedestrian Detection');
    end
    imshow(imgfr);
    hold on;

    % Compute the association matrix
    if regnum
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
        
        [L_gt, num_gt] = bwlabel(groundTruthMatrix(:,:, i));    % Label GT objects
        [L_det, num_det] = bwlabel(DetectorMatrix);            % Label detected objects
        
        C = zeros(num_gt, num_det); % Initialize association matrix
    
        for l = 1:num_gt
            for m = 1:num_det
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
                end
            end
        end
        % disp(C) % So é necessario contabilizar o numero de merges
                
        % drawGT(i, groundTruth, str, pathToImages, extName);

        disp(pedestrianDb)
        for j=1:regnum
            maxMatchScore = -inf;  % Reset for each detection
            best_match_idx = -1;
            currentID = [];
            [lin col]= find(lb == inds(j));
            upLPoint = min([lin col]);
            dWindow  = max([lin col]) - upLPoint + 1;      

            rectangle('Position',[fliplr(upLPoint) fliplr(dWindow)],'EdgeColor',[1 1 0],...
                'linewidth',2);

            %trajectoryFrame = trajectoryFrame + 1; 
            %if trajectoryFrame == 20
                %trajectoryFrame = 1;
            %end

            %delete(centroids(:, trajectoryFrame));
            if regionProps(inds(j)).Area > 2300
                continue;
            end

            centroid = regionProps(inds(j)).Centroid;
            bbox = regionProps(inds(j)).BoundingBox;
            
            x_min = floor(bbox(1)) + 1;
            y_min = floor(bbox(2)) + 1;
            x_max = min(width, floor(bbox(1) + bbox(3)));
            y_max = min(height, floor(bbox(2) + bbox(4)));

            % Crop the detection region, fica so mm o crop perfeitinho
            croppedRegion = imgfr(y_min:y_max, x_min:x_max, :);
            
            % Compute histograms for each channel
            numBins = 256;
            histR = histcounts(croppedRegion(:,:,1), numBins);
            histG = histcounts(croppedRegion(:,:,2), numBins);
            histB = histcounts(croppedRegion(:,:,3), numBins);

            % Normalize histograms (optional)
            histR = histR / sum(histR);
            histG = histG / sum(histG);
            histB = histB / sum(histB);

            max_iou = 0;
            best_match_idx = -1;
            similarityTreshold = 0.7; % Tem de ser menor que isto
            distanceThreshold = 100;
            
            for k = 1:length(pedestrianDb)

                % Compute distance so all the pedestrians already in DB with Treshold

                % If not there already than add to db with centroid and new ID
    
                % If it is there we should update centroid and histogram and
                % keep the label
                    
                

                % IoU entre bbox de dois frames consecutivos
                iou = computeIoU(bbox, pedestrianDb(k).BoundingBox);
                bDistance = bhattacharyya(pedestrianDb(k).Histogram, [histR, histG, histB]);
                centroidDist = norm(centroid - pedestrianDb(k).Centroid);
                % IoU threshold (like discussed during the labs)
                if bDistance < similarityTreshold && centroidDist < distanceThreshold
                    matchScore =  (1 - bDistance) + (1 - (centroidDist));
                    
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
                pedestrianDb(best_match_idx).Histogram = [histR, histG, histB];

                pedestrianDb(best_match_idx).Trajectory = [pedestrianDb(best_match_idx).Trajectory; centroid];
                if size(pedestrianDb(best_match_idx).Trajectory, 1) > max_trajec_len
                    pedestrianDb(best_match_idx).Trajectory(1, :) = []; % we need to remove oldest point in order to get the trail
                end
            else
                currentID = next_id;
                pedestrianDb(end + 1) = struct("ID", next_id, "Centroid", centroid, ...
                                             "BoundingBox", bbox, "Trajectory", centroid, ...
                                             "Histogram", [histR, histG, histB]);
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
            text(textPosition(1), textPosition(2), sprintf('ID: %d', currentID), 'Color', 'yellow', ...
            'FontSize', 10, 'FontWeight', 'bold');

            %Draw heatmaps
            if drawHeatmap
                centroidHeatmap = round(centroid);
                xH = min(max(centroidHeatmap(1), 1), width);
                yH = min(max(centroidHeatmap(2), 1), height);
    
                [XH, YH] = meshgrid(1:width, 1:height);
                gaussian = exp(-((XH - xH).^2 + (YH - yH).^2) / (2 * sigma^2));
        
                heatmap = heatmap + gaussian;
    
                dynamicHeatmap = dynamicHeatmap + gaussian;
                %Apply decay factor for dynamic heatmap
                dynamicHeatmap = dynamicHeatmap * heatmapDecay;
            end

        end
  
    end

    if drawHeatmap
        subplot(2, 2, 2);
        imshow(heatmap, []);
        colormap("jet");  
        colorbar;
        title('Static Heatmap');
    
        subplot(2, 2, 3);
        imshow(dynamicHeatmap, []);
        colormap("jet"); 
        colorbar; 
        title('Dynamic Heatmap');
    end
    drawnow;
end

% EM algorithm
X = [];
for i = 1:length(pedestrianDb)
    X = [X; pedestrianDb(i).Trajectory];
end

K = 3;

if ~isempty(X)
    % fitgmdist performs EM using k-means for parameter init; regularizationValue
    % pervents issues (outliers, etc)
    gmmModel = fitgmdist(X, K, 'RegularizationValue', 0.1); 
    clusterLabels = cluster(gmmModel, X);
    figure; hold on;
    colors = ['r', 'g', 'b', 'y', 'm'];
    for k = 1:K
        scatter(X(clusterLabels == k, 1), X(clusterLabels == k, 2), 10, colors(k), 'filled');
    end
    title('Pedestrian Trajectory Clusters');
    xlabel('X Position'); ylabel('Y Position');
    legend('Cluster 1', 'Cluster 2', 'Cluster 3');
end


