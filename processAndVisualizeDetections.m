function processAndVisualizeDetections(vid4D, detector, groundTruth)
    figure, hold on;
    
    personBoxes = cell(size(vid4D, 4), 1);
    
    for i = 1:size(vid4D, 4)
        imgfr = vid4D(:,:,:,i);

        gtForFrame = groundTruth(groundTruth(:,1) == i, :);
        

        [bboxes, scores, labels] = detect(detector, imgfr, Threshold=0.4);
        personBoxes{i} = bboxes(labels=="person", :);
        

        detectedImg = imgfr;
        

        detectedImg = insertObjectAnnotation(detectedImg, "Rectangle", personBoxes{i}, "person", 'Color', 'green');
        

        gtBoxes = gtForFrame(:, 3:6); 
        gtLabels = arrayfun(@(id) sprintf('GT-%d', id), gtForFrame(:, 2), 'UniformOutput', false);
        detectedImg = insertObjectAnnotation(detectedImg, "Rectangle", gtBoxes, gtLabels, 'Color', 'red');
        

        imshow(detectedImg);
        title(sprintf('Frame %d - Green: Detections, Red: Ground Truth', i));
        drawnow;
        

    end
end