function [falsePositiveRate, falseNegativeRate] = computeFPFN(vid4D, detector, groundTruth, iouThreshold)
   
    if nargin < 4
        iouThreshold = 0.5;
    end
    
    totalGT = 0;
    totalDetections = 0;
    totalTP = 0;  
    totalFP = 0; 
    totalFN = 0; 
    

    fpFrames = [];
    fnFrames = [];
    

    for frame = 1:size(vid4D, 4) / 2
       
        gtForFrame = groundTruth(groundTruth(:,1) == frame, :);
        numGT = size(gtForFrame, 1);
        totalGT = totalGT + numGT;
        
      
        imgfr = vid4D(:,:,:,frame);
        
      
        [bboxes, scores, labels] = detect(detector, imgfr, Threshold=0.4);
        personBoxes = bboxes(labels=="person", :);
        numDetections = size(personBoxes, 1);
        totalDetections = totalDetections + numDetections;
        
       
        regionProps = struct('BoundingBox', cell(1, numDetections));
        for i = 1:numDetections
            regionProps(i).BoundingBox = personBoxes(i, :);
        end
        
       
        associationMatrix = computeAssociationMatrix(groundTruth, regionProps, 1:numDetections, frame, iouThreshold);
        
        
        tp = sum(sum(associationMatrix, 2) > 0); 
        fn = numGT - tp;                         
        fp = numDetections - sum(sum(associationMatrix, 1) > 0);
        
        totalTP = totalTP + tp;
        totalFN = totalFN + fn;
        totalFP = totalFP + fp;
        
        
        if fp > 0
            fpFrames = [fpFrames; frame];
        end
        if fn > 0
            fnFrames = [fnFrames; frame];
        end
    end
    

    if totalGT > 0
        falseNegativeRate = totalFN / totalGT;
    else
        falseNegativeRate = 0;
    end
    
    if totalDetections > 0
        falsePositiveRate = totalFP / totalDetections;
    else
        falsePositiveRate = 0;
    end
    

    fprintf('Evaluation at IoU threshold %.2f:\n', iouThreshold);
    fprintf('Total ground truth objects: %d\n', totalGT);
    fprintf('Total detections: %d\n', totalDetections);
    fprintf('True positives: %d\n', totalTP);
    fprintf('False positives: %d (%.2f%%)\n', totalFP, falsePositiveRate*100);
    fprintf('False negatives: %d (%.2f%%)\n', totalFN, falseNegativeRate*100);
    
end