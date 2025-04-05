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
        [successPercentage, associationMatrixCellArray{i}] = computeAssociationMatrix(groundTruthMatrix, regionProps, inds, height, width, i, regnum);
        successPercentageArray = [successPercentageArray, successPercentage];
        
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
    %pause(1);
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


% % Get delta positions from pedestrian trajectories
% deltaTraj = [];
% for i = 1:length(pedestrianDB)
%     traj = pedestrianDB(i).Trajectory;
%     if size(traj, 1) > 1
%         delta = traj(end, :) - traj(1, :);
%         deltaTraj = [deltaTraj; delta];
%     end
% end
% 
% % Number of Gaussian components
% K = 2; % Changeable
% 
% % Initialize parameters
% [N, D] = size(deltaTraj);
% rng(1); % for reproducibility
% mu = deltaTraj(randperm(N, K), :); % Random means
% Sigma = zeros(D, D, K); % Covariance matrices
% for k = 1:K
%     Sigma(:, :, k) = eye(D); % Identity covariance matrix
% end
% pi_k = ones(1, K) / K; % Uniform mixing coefficients
% log_likelihood = [];
% 
% maxIter = 100;
% tol = 1e-6;
% 
% for iter = 1:maxIter
%     % E-step
%     gamma = zeros(N, K); % responsibilities
%     for k = 1:K
%         for n = 1:N
%             diffVec = deltaTraj(n, :) - mu(k, :);
%             SigmaReg = Sigma(:, :, k) + 1e-6 * eye(D);
%             gamma(n, k) = pi_k(k) * exp(-0.5 * diffVec * inv(SigmaReg) * diffVec') / sqrt(det(SigmaReg));
%         end
%     end
%     gamma = gamma ./ sum(gamma, 2); % normalize responsibilities
% 
%     % M-step
%     Nk = sum(gamma, 1);
%     for k = 1:K
%         mu(k, :) = sum(gamma(:, k) .* deltaTraj) / Nk(k);
%         X_centered = deltaTraj - mu(k, :);
%         SigmaReg = (X_centered' * (X_centered .* gamma(:, k))) / Nk(k) + 1e-6 * eye(D);
%         Sigma(:, :, k) = SigmaReg;
%     end
%     pi_k = Nk / N;
% 
%     % Check for convergence (log likelihood)
%     ll = 0;
%     for n = 1:N
%         for k = 1:K
%             diffVec = deltaTraj(n, :) - mu(k, :);
%             SigmaReg = Sigma(:, :, k) + 1e-6 * eye(D);
%             ll = ll + log(pi_k(k) * exp(-0.5 * diffVec * inv(SigmaReg) * diffVec') / sqrt(det(SigmaReg)));
%         end
%     end
%     log_likelihood(end + 1) = ll;
%     if iter > 1 && abs(log_likelihood(end) - log_likelihood(end - 1)) < tol
%         break;
%     end
% end
% 
% % Assign clusters
% [~, clusterLabels] = max(gamma, [], 2);
% 
% % Plot results
% figure; hold on;
% colors = lines(K);
% for k = 1:K
%     scatter(deltaTraj(clusterLabels == k, 1), deltaTraj(clusterLabels == k, 2), 10, colors(k, :), 'filled');
% end
% title('GMM Clustering on Pedestrian Delta Trajectories');
% xlabel('\DeltaX'); ylabel('\DeltaY');
% legend(arrayfun(@(k) sprintf('Cluster %d', k), 1:K, 'UniformOutput', false));

% Build displacement data
displacementTraj = [];
for i = 1:length(pedestrianDB)
    traj = pedestrianDB(i).Trajectory;
    if size(traj, 1) == 10
        dispVecs = diff(traj);        % 9x2 displacements
        dispFlat = dispVecs(:)';      % 1x18 vector
        displacementTraj = [displacementTraj; dispFlat];
    end
end

% Normalize
displacementTraj = zscore(displacementTraj);
[N, D] = size(displacementTraj);

% Set K = 2
K = 2;

% Initialize parameters
rng(1);
mu = displacementTraj(randperm(N, K), :);
Sigma = repmat(eye(D), [1, 1, K]);
pi_k = ones(1, K) / K;
log_likelihood = [];

maxIter = 100;
tol = 1e-6;

for iter = 1:maxIter
    % E-step
    gamma = zeros(N, K);
    for k = 1:K
        for n = 1:N
            diffVec = displacementTraj(n, :) - mu(k, :);
            SigmaReg = Sigma(:, :, k) + 1e-6 * eye(D);
            gamma(n, k) = pi_k(k) * exp(-0.5 * diffVec * (SigmaReg \ diffVec')) / sqrt(det(SigmaReg));
        end
    end
    gamma = gamma ./ sum(gamma, 2);

    % M-step
    Nk = sum(gamma, 1);
    for k = 1:K
        mu(k, :) = sum(gamma(:, k) .* displacementTraj) / Nk(k);
        X_centered = displacementTraj - mu(k, :);
        Sigma_k = zeros(D);
        for n = 1:N
            Sigma_k = Sigma_k + gamma(n, k) * (X_centered(n, :)' * X_centered(n, :));
        end
        Sigma(:, :, k) = Sigma_k / Nk(k) + 1e-6 * eye(D);
    end
    pi_k = Nk / N;

    % Log-likelihood
    ll = 0;
    for n = 1:N
        temp = 0;
        for k = 1:K
            diffVec = displacementTraj(n, :) - mu(k, :);
            SigmaReg = Sigma(:, :, k) + 1e-6 * eye(D);
            temp = temp + pi_k(k) * exp(-0.5 * diffVec * (SigmaReg \ diffVec')) / sqrt(det(SigmaReg));
        end
        ll = ll + log(temp);
    end
    log_likelihood(end + 1) = ll;
    if iter > 1 && abs(log_likelihood(end) - log_likelihood(end - 1)) < tol
        break;
    end
end

% Assign clusters
[~, clusterLabels] = max(gamma, [], 2);

% Plot results
figure; hold on;
colors = lines(K); % Generate K distinct colors
for k = 1:K
    scatter(displacementTraj(clusterLabels == k, 1), displacementTraj(clusterLabels == k, 2), 10, colors(k, :), 'filled');
end
title('GMM Clustering on Pedestrian Trajectories');
xlabel('\DeltaX'); ylabel('\DeltaY');
legend(arrayfun(@(k) sprintf('Cluster %d', k), 1:K, 'UniformOutput', false));


