bbox1 = [274.500000000000,217.500000000000,34,86];
bbox2 = [259.500000000000,220.500000000000,48,88];

disp(computeIoU(bbox1, bbox2));

hist1 = pedestrianDb(4).Histogram;
hist2 = pedestrianDb(9).Histogram;

disp(bhattacharyya(hist1, hist2));