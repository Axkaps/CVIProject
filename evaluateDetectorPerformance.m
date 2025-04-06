function evaluateDetectorPerformance(vid4D, detector, groundTruth)

    thresholds = 0:0.1:1.0;
    

    successRates = zeros(size(thresholds));
    totalFramesWithGT = 0;
    

    for frame = 1:size(vid4D, 4)

        gtForFrame = groundTruth(groundTruth(:,1) == frame, :);
        

        if isempty(gtForFrame)
            continue;
        end
        
        totalFramesWithGT = totalFramesWithGT + 1;
        

        imgfr = vid4D(:,:,:,frame);
        

        [bboxes, scores, labels] = detect(detector, imgfr, Threshold=0.4);
        personBoxes = bboxes(labels=="person", :);
        
        regionProps = struct('BoundingBox', cell(1, size(personBoxes, 1)));
        for i = 1:size(personBoxes, 1)
            regionProps(i).BoundingBox = personBoxes(i, :);
        end
        

        for t = 1:length(thresholds)

            associationMatrix = computeAssociationMatrix(groundTruth, regionProps, 1:length(regionProps), frame, thresholds(t));
            
            numSuccessful = sum(sum(associationMatrix, 2) > 0);
            successRates(t) = successRates(t) + numSuccessful / size(gtForFrame, 1);
        end
    end
    

    if totalFramesWithGT > 0
        successRates = successRates / totalFramesWithGT;
    end
    

    plotSuccessRates(thresholds, successRates);

end

