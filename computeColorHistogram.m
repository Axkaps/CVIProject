function histFeature = computeColorHistogram(regionImage)
    numBins = 256;


    histR = histcounts(regionImage(:,:,1), numBins);
    histG = histcounts(regionImage(:,:,2), numBins);
    histB = histcounts(regionImage(:,:,3), numBins);

    histR = histR / sum(histR);
    histG = histG / sum(histG);
    histB = histB / sum(histB);

    histFeature = [histR, histG, histB];
end