

groundTruthMatrix = getGTMatrix(groundTruth, numFrames);


if max(groundTruthMatrix(:,:,i), [], 'all') == 0
    disp(['Frame ', num2str(i), ' is completely black!']);
end

% figure, hold on;
% for i=1:numFrames
%     i
%     imshow(groundTruthMatrix(:,:,i));
% end