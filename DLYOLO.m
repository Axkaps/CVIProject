
% Initialize YOLO detector
detector = yolov4ObjectDetector("csp-darknet53-coco");

% Process video and store detections
% processAndVisualizeDetections(vid4D, detector, groundTruth);

% Evaluate detector performance and generate success plot
% evaluateDetectorPerformance(vid4D, detector, groundTruth);

% Compute false positives and false negatives at standard IoU threshold (0.5)
[fpRate, fnRate] = computeFPFN(vid4D, detector, groundTruth, 0.7);

% Generate success plot for different thresholds
% generateSuccessPlot(vid4D, detector, groundTruth);

