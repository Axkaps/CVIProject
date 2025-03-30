thr = 50;
minArea = 250;
maxArea = 2500;
se = strel('disk', 5);
next_id = 20;
height = 576;
width = 768;
sigma = 20;
heatmap = zeros(height, width);
dynamicHeatmap = zeros(height, width);
heatmapDecay = 0.99;
associationMatrixCellArray = {}; % Empty cell array

assigned = [];


successPercentage = 0;
successPercentageArray = [];


% Load the db I have
load('pedestrianDB.mat', 'pedestrianDb');

if evaluatePerformance
    thresholds = 0:0.1:1;
    successRates = zeros(size(thresholds));

    subplot(2, 2, 2);
    successPlot = plot(thresholds, successRates, '-o', 'LineWidth', 2);
    xlabel('IoU Threshold');
    ylabel('Success Rate');
    title('Success Plot');
    grid on;
end

for i=1:size(vid4D, 4)
    assigned = [];
    imgfr = vid4D(:,:,:,i);
    imgdif = (abs(double(bkg(:,:,1))-double(imgfr(:,:,1))) > thr) | ...
        (abs(double(bkg(:,:,2))-double(imgfr(:,:,2))) > thr) | ...
        (abs(double(bkg(:,:,3))-double(imgfr(:,:,3))) > thr);

    bw = imgdif; 


    % Use lb matrix to be my detector matrix
    [lb num] = bwlabel(bw); 
    regionProps = regionprops(lb, 'Area', 'BoundingBox', 'Centroid');
    inds = find([regionProps.Area] > minArea & [regionProps.Area] < maxArea);
    regnum = length(inds);
    

    if drawHeatmap || evaluatePerformance
        subplot(2, 2, 1);
        title('Pedestrian Detection');
    end
    imshow(imgfr); hold on;

    % Compute the association matrix
    if regnum
        [successPercentage, associationMatrixCellArray{i}] = computeAssociationMatrix(groundTruthMatrix, regionProps, inds, height, width, i, regnum);
        successPercentageArray = [successPercentageArray, successPercentage];

        for j=inds
            currentID = [];
            [lin col]= find(lb == j);
            upLPoint = min([lin col]);
            dWindow  = max([lin col]) - upLPoint + 1;      


            % Detect pedestrian and assign ID
            [pedestrianDB, currentID, next_id] = pedestrianDetection(pedestrianDB, imgfr, regionProps, j, next_id, assigned);
            assigned(end + 1) = currentID;
            if drawTrajectory
                plot(regionProps(j).Centroid(1), regionProps(j).Centroid(2), 'g.', 'MarkerSize', 20);
                %TODO: Fix error when running point 3
                if best_match_idx ~= -1
                    traj = pedestrianDB(best_match_idx).Trajectory;
                    if size(traj, 1) > 1
                        plot(traj(:, 1), traj(:, 2), 'g-', 'LineWidth', 2); 
    
                    end
                end
            end

            
            textPosition = [fliplr(upLPoint)  - [0, 10]];
            %Display bbox
            rectangle('Position',[fliplr(upLPoint) fliplr(dWindow)],'EdgeColor',[1 1 0],...
                'linewidth',2);
            % Display pedestrian ID
            text(textPosition(1), textPosition(2), sprintf('ID: %d', currentID), 'Color', 'yellow', ...
            'FontSize', 10, 'FontWeight', 'bold');

            %Draw heatmaps
            if drawHeatmap
                centroidHeatmap = round(regionProps(j).Centroid);
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
    elseif evaluatePerformance
        for k = 1:length(thresholds)
            successRates(k) = sum(successPercentageArray >= thresholds(k)) / length(successPercentageArray);
        end
        set(successPlot, 'YData', successRates);
    end

    drawnow;
    hold off;
    pause(1);
end

% EM algorithm
X = [];
for i = 1:length(pedestrianDB)
    X = [X; pedestrianDB(i).Trajectory];
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


