function generateSuccessPlot(vid4D, detector, groundTruth)

    thresholds = 0:0.1:1.0;
    successRates = zeros(size(thresholds));
    

    for i = 1:length(thresholds)
        threshold = thresholds(i);
        
        totalGtObjects = 0;
        totalSuccessful = 0;
        

        for frame = 1:size(vid4D, 4)

            gtForFrame = groundTruth(groundTruth(:,1) == frame, :);
            numGT = size(gtForFrame, 1);
            totalGtObjects = totalGtObjects + numGT;
            
            if numGT == 0
                continue;
            end
            

            imgfr = vid4D(:,:,:,frame);
            [bboxes, scores, labels] = detect(detector, imgfr, Threshold=0.4);
            personBoxes = bboxes(labels=="person", :);
            
            % Create regionProps structure
            regionProps = struct('BoundingBox', cell(1, size(personBoxes, 1)));
            for j = 1:size(personBoxes, 1)
                regionProps(j).BoundingBox = personBoxes(j, :);
            end
            

            associationMatrix = computeAssociationMatrix(groundTruth, regionProps, 1:length(regionProps), frame, threshold);
            
            numSuccessful = sum(sum(associationMatrix, 2) > 0);
            totalSuccessful = totalSuccessful + numSuccessful;
        end
        
        if totalGtObjects > 0
            successRates(i) = totalSuccessful / totalGtObjects;
        else
            successRates(i) = 0;
        end
    end
    

    figure;
    plot(thresholds, successRates, 'LineWidth', 2);
    grid on;
    xlabel('IoU Threshold');
    ylabel('Success Rate');
    title('Success Plot for Pedestrian Detection');
    
end