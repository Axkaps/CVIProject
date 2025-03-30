function [vid4D, bkg] = calculateBackground(numFrames, str, pathToImages, extName)
 
    step = 1; i = 1;
    
    %figure; hold on,
    for k =1:step:numFrames
        img= imread(sprintf(str,pathToImages,k-1,extName));   
        
        vid4D(:,:,:,i) = img;
        %imshow(img); drawnow
        i = i+1;

        if mod(k, 100) == 0
            disp("...");
        end
    end
    
    % Get the background (Should try both median and low pass i guess)
    bkg = median(vid4D,4);

end


