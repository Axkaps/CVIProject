thr = 50;
minArea = 200;
maxArea = 7000;
ID = 1;
se = strel('rectangle', [3 10]);

%maxpeople, maxframes
%centroids = zeros(15, 20); 
%trajectoryFrame = 0;

figure; hold on
for i=1:size(vid4D, 4)
    imgfr = vid4D(:,:,:,i);
    imgdif = (abs(double(bkg(:,:,1))-double(imgfr(:,:,1))) > thr) | ...
        (abs(double(bkg(:,:,2))-double(imgfr(:,:,2))) > thr) | ...
        (abs(double(bkg(:,:,3))-double(imgfr(:,:,3))) > thr);

    % bw1 = imclose(imgdif,se);
    % bw2 = imerode(bw1,se);
    bw = imgdif;
    % bw = imerode(bw, se); % Reduce objects trying to mitigate double detection
    % bw = imdilate(bw, se); % tentativa
   
    [lb num]=bwlabel(bw);
    regionProps = regionprops(lb,'area','FilledImage','Centroid');
    inds = find([regionProps.Area] > minArea & [regionProps.Area] < maxArea);
    
    regnum = length(inds);
    imshow(imgfr);
    if regnum
        for j=1:regnum
            [lin col]= find(lb == inds(j));
            upLPoint = min([lin col]);
            dWindow  = max([lin col]) - upLPoint + 1;
            
            
            rectangle('Position',[fliplr(upLPoint) fliplr(dWindow)],'EdgeColor',[1 1 0],...
                'linewidth',2);

            %trajectoryFrame = trajectoryFrame + 1; 
            %if trajectoryFrame == 20
                %trajectoryFrame = 1;
            %end

            %delete(centroids(:, trajectoryFrame));

            %r = plot(regionProps(inds(j)).Centroid(1),regionProps(inds(j)).Centroid(2),'g.','markersize',20);

            %centroids(j, trajectoryFrame) = r;
            
            textPosition = [fliplr(upLPoint)  - [0, 10]];
            % Display pedestrian ID
            text(textPosition(1), textPosition(2), sprintf('ID: %d', ID), 'Color', 'yellow', ...
            'FontSize', 10, 'FontWeight', 'bold');
            ID = ID + 1;
        end
        ID = 1; % Provisory labels
    end
    drawnow
end