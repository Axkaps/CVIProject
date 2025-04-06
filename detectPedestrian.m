thr = 50;
minArea = 200;
maxArea = 4500;
se = strel('disk', 5);
next_id = 21;
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

totalFN = 0;
totalFP = 0;
totalGT = 0;
totalDT = 0; 

% Create successPlot of IoU
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

    % Inside the for loop where regionProps are processed
    if regnum
        %[successPercentage, associationMatrixCellArray{i}] = computeAssociationMatrix(groundTruthMatrix, regionProps, inds, height, width, i, regnum);
        associationMatrixCellArray{i} = computeAssociationMatrix(groundTruth, regionProps, inds, i, 0.15);

        % successPercentageArray = [successPercentageArray, successPercentage];
        
        for filtered_idx=1:length(inds)
            
            j = inds(filtered_idx);
            currentID = [];
            [lin col]= find(lb == j);
            upLPoint = min([lin col]);
            dWindow  = max([lin col]) - upLPoint + 1;      
    
            % Get the current association matrix for this frame
            currentAssociationMatrix = [];
            if ~isempty(associationMatrixCellArray{i})
                currentAssociationMatrix = associationMatrixCellArray{i};
            end
    
            % Detect pedestrian and assign ID, passing the association matrix
            [pedestrianDB, currentID, next_id] = pedestrianDetection(pedestrianDB, imgfr, regionProps, j, next_id, assigned, currentAssociationMatrix, filtered_idx);
            
            % Only process further if not a merge situation (currentID != -1)
            if currentID ~= -1
                
                assigned(end + 1) = currentID;
                best_match_idx = currentID; 
                
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
            else
                textPosition = [fliplr(upLPoint)  - [0, 10]];
                %Display bbox
                rectangle('Position',[fliplr(upLPoint) fliplr(dWindow)],'EdgeColor',[1 1 0],...
                    'linewidth',2);
                % Display pedestrian ID
                text(textPosition(1), textPosition(2), sprintf('ID: %d', currentID), 'Color', 'yellow', ...
                'FontSize', 10, 'FontWeight', 'bold');
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

        if ~isempty(associationMatrixCellArray{i})
            currentFN = sum(all(associationMatrixCellArray{i} == 0, 2));
            totalFN = totalFN + currentFN;
    
            currentFP = sum(all(associationMatrixCellArray{i} == 0, 1));
            totalFP = totalFP + currentFP;
    
            totalGT = totalGT + size(associationMatrixCellArray{i}, 1); 
            totalDT = totalDT + size(associationMatrixCellArray{i}, 2); 
        end

        subplot(2, 2, 3);
        bar(1, totalFN, 'r'); 
        ylabel('Ground Truth detections');
        xlabel('Total False Negatives (FN)');
        title('Total FN in Ground Truth detections');
        xticks(1);
        xticklabels({sprintf('Total FN: %d', totalFN)});
        ylim([0, max(totalFN, 1.2 * totalGT)]); 
        grid on;
        
        subplot(2, 2, 4);
        bar(1, totalFP, 'b');
        ylabel('Algorithm detections');
        xlabel('Total False Positives (FP)');
        title('Total FP in Algorithm detections');
        xticks(1);
        xticklabels({sprintf('Total FP: %d', totalFP)});
        ylim([0, max(totalFP, 1.2 * totalDT)]); 
        grid on;
    end

    drawnow;
    hold off;
    pause(0.5);
end

%Calculate FN and FP percentages
%Count the total of lines of zeros dividing by total columns of associationMatrixCellArray
%for i = 1:length(associationMatrixCellArray)
%    if ~isempty(associationMatrixCellArray{i}) 
        %C = associationMatrixCellArray{i}; 

       % currentFN = sum(all(C == 0, 2));
      %  totalFN = totalFN + currentFN;

     %   currentFP = sum(all(C == 0, 1));
    %    totalFP = totalFP + currentFP;

   %     totalGT = totalGT + size(C, 1); 
  %      totalDT = totalDT + size(C, 2); 
 %   end
%end

%percentageFN = (totalFN / totalGT) * 100;
%percentageFP = (totalFP / totalDT) * 100;


% fprintf('False Negative Percentage: %.2f%%\n', percentageFN);
% fprintf('False Positive Percentage: %.2f%%\n', percentageFP);


% Load data (assumes format: frame, ID, x, y)
data = load('.\PETS-S2L1\gt\gt.txt');

% Organize trajectories by track ID
track_ids = unique(data(:, 2));
trajectories = cell(length(track_ids), 1);

for i = 1:length(track_ids)
    traj = data(data(:, 2) == track_ids(i), [1, 3, 4]); % [frame, x, y]
    trajectories{i} = traj(:, 2:3); % only (x, y)
end

allDisplacements = [];

for i = 1:size(X, 1)
    x = trajectories{i}(:, 1);
    y = trajectories{i}(:, 2);
    dx = diff(x);
    dy = diff(y);
    D = [dx, dy];
    allDisplacements = [allDisplacements; D];
end

% Run EM-GMM on displacement vectors
K = 4;
max_iters = 1000;
[idx, mu_init] = kmeans(allDisplacements, K);

[mu, Sigma, pi_k, gamma, log_likelihoods] = em_gmm(allDisplacements, K, max_iters, mu_init, idx);
[~, labels] = max(gamma, [], 2);

plot(log_likelihoods(1:max_iters));
xlabel('Iteration');
ylabel('Log-Likelihood');
title('EM-GMM Convergence');

colors = lines(K);  % distinct colors

figure;
hold on;

% 1. Plot the displacement vectors
gscatter(allDisplacements(:,1), allDisplacements(:,2), labels);
title('EM-GMM Displacement Clustering');
xlabel('dx'); ylabel('dy');
axis equal; grid on;

% 2. Plot each Gaussian
for k = 1:K
    plot_gaussian(mu(k,:), Sigma(:,:,k), colors(k,:));
end

legend('Cluster 1', 'Cluster 2', 'Cluster 3', 'Cluster 4', 'Gaussians');


% Plot clustered trajectories
figure; hold on;
colors = lines(K);
for i = 1:size(X, 1)
    x = X(i, 1:n_points);
    y = X(i, n_points+1:end);
    plot(x, y, 'o', 'Color', colors(labels(i), :), 'MarkerSize', 5);
end
title('Clustered Trajectories from EM-GMM');
xlabel('X'); ylabel('Y'); axis equal; grid on;



