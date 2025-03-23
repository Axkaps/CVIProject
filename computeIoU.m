function iou = computeIoU(boxA, boxB)
    xA = max(boxA(1), boxB(1)); % leftmost edge of inter area
    yA = max(boxA(2), boxB(2)); % biggest y value (since we'll add the other box's height to its y value)
    xB = min(boxA(1) + boxA(3), boxB(1) + boxB(3)); % minimum since we want to know where's the rightmost edge of the inter area
    yB = min(boxA(2) + boxA(4), boxB(2) + boxB(4)); % we later get the height of the inter area

    interArea = max(0, xB - xA) * max(0, yB - yA); % cant be negative, which could happen if there was no overlap
    boxAArea = boxA(3) * boxA(4);
    boxBArea = boxB(3) * boxB(4);
    
    iou = interArea / (boxAArea + boxBArea - interArea);
end