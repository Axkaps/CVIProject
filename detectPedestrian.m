thr = 50;
minArea = 100;
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
associationMatrixCellArray = {}; % Empty cell array

%maxpeople, maxframes
%centroids = zeros(15, 20); 
%trajectoryFrame = 0;


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
    imshow(lb); hold on;
    regnum = length(inds);
    

    if drawHeatmap
        subplot(2, 2, 1);
        title('Pedestrian Detection');
    end
    %imshow(imgfr); hold on;

    % Compute the association matrix
    if regnum
        associationMatrixCellArray{i} = computeAssociationMatrix(groundTruthMatrix, regionProps, inds, height, width, i, regnum);
        
        
        % disp(C) % So é necessario contabilizar o numero de merges

        for j=1:regnum
            maxMatchScore = -inf;  % Reset for each detection
            best_match_idx = -1;
            currentID = [];
            [lin col]= find(lb == inds(j));
            upLPoint = min([lin col]);
            dWindow  = max([lin col]) - upLPoint + 1;      

            rectangle('Position',[fliplr(upLPoint) fliplr(dWindow)],'EdgeColor',[1 1 0],...
                'linewidth',2);

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

            histR = histR / sum(histR);
            histG = histG / sum(histG);
            histB = histB / sum(histB);

            max_iou = 0;
            best_match_idx = -1;
            similarityTreshold = 0.7; % Tem de ser menor que isto
            distanceThreshold = 100;
            
            for k = 1:length(pedestrianDb)                    

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
    hold off;
end

% EM algorithm
X = [];
for i = 1:length(pedestrianDb)
    X = [X; pedestrianDb(i).Trajectory];
end

K = 1;

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


