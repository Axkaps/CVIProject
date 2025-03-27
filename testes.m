figure, hold on;



imgToProcess = vid4D(:,:,:,198);


imgdif = (abs(double(bkg(:,:,1))-double(imgToProcess(:,:,1))) > thr) | ...
        (abs(double(bkg(:,:,2))-double(imgToProcess(:,:,2))) > thr) | ...
        (abs(double(bkg(:,:,3))-double(imgToProcess(:,:,3))) > thr);

[lb, num] = bwlabel(imgdif);
regionProps = regionprops(lb, 'Area', 'BoundingBox', 'Centroid');
inds = find([regionProps.Area] > minArea & [regionProps.Area] < maxArea); % gotta do a new one for the split version

regnum = length(inds);
disp(inds);
for i = inds
    
    disp(regionProps(i).Area);

end

% Select the first detected region
k = 1; %52 53
singleRegionMask = (lb == k);

% Get bounding box
stats = regionprops(singleRegionMask, 'BoundingBox');
bbox = stats.BoundingBox;
padding = 10;
bbox = [
    max(1, bbox(1) - padding), % x_min (avoid going out of image)
    max(1, bbox(2) - padding), % y_min
    min(bbox(3) + 2*padding, size(imgToProcess, 2) - bbox(1)), % width
    min(bbox(4) + 2*padding, size(imgToProcess, 1) - bbox(2))  % height
];

% Crop the original image
croppedImage = imcrop(imgToProcess, bbox);

% (Optional) Apply the mask to the cropped region
croppedMask = imcrop(singleRegionMask, bbox);
maskedImage = croppedImage;
maskedImage(repmat(~croppedMask, [1, 1, 3])) = 0; % For RGB images

% Display results
subplot(1, 3, 1); imshow(lb); title('Original Image');
subplot(1, 3, 2); imshow(croppedImage); title('Cropped Region');
subplot(1, 3, 3); imshow(maskedImage); title('Masked Region');


