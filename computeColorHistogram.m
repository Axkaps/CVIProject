function histFeature = computeColorHistogram(regionImage)
    numBins = 256;

    % Compute histograms with correct bin count
    histR = histcounts(regionImage(:,:,1), numBins);
    histG = histcounts(regionImage(:,:,2), numBins);
    histB = histcounts(regionImage(:,:,3), numBins);

    % Normalize and avoid division by zero
    epsilon = 1e-10;
    histR = histR / (sum(histR) + epsilon);
    histG = histG / (sum(histG) + epsilon);
    histB = histB / (sum(histB) + epsilon);

    % Concatenate histogram features
    histFeature = [histR, histG, histB];
end
