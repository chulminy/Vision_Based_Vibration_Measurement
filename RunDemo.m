%% Vision based vibration measurement using markers
%
%% Contact
%  Name  : Chul Min Yeum
%  Email : chulminy@gmail.com
%  Please contact me if you have a question or find a bug

%% Configuration
%
% We will configure several parameters related to intial locations of all
% targets, skeleton figures, thresholds for target detection, and video
% recoding. Activating "DEBUG" (DEBUG=1) will be helpful for correct
% configuration. 
%
clear; close all; clc; warning('off');

% result folder
folderOut = 'result';

% DEBUG is 1 if you want to plot intermidiate results
DEBUG = 0; 

% get a list of all jpg files in an image directory
imgPath      = dir('img\*.jpg');
imgPath      = cellfun(@(C)fullfile('img', C),{imgPath.name},'uni',false);

% total # of images
nImg            = length(imgPath);  

% image sizes
imgSize = [1300 2046];

% initial locations of target points
iniLocH         = [1904 1420 931 471];                  % height
iniLocW         = [148 310 466 625 778 940 1111];       % width
targetStripH    = 100;
targetSize      = 20;

% plot initial target locations
if DEBUG
    imgTmp = imread(imgPath{1});
    
    circleLoc = [repmat(iniLocW',4,1)...
        reshape(repmat(iniLocH,7,1),28,1) ...
        ones(28,1)*targetSize/2];
    
    imgTmp = insertShape(imgTmp,'circle',int32(circleLoc), ...
        'color','red','Linewidth',3);
    
    figure(1); imshow(imgTmp);
    clearvars img circleLoc;
end

% point connection map
ptH = [1 2;2 3;3 4;4 5;5 6;6 7; ...
       8 9;9 10;10 11;11 12;12 13;13 14; ...
       15 16;16 17;17 18;18 19;19 20;20 21; ...
       22 23;23 24;24 25;25 26;26 27;27 28;];
         
ptVtmp = [1 8; 8 15; 15 22];
ptV    = [ptVtmp;ptVtmp+3;ptVtmp+6]; clearvars ptVtmp;

ptMap  = [ptH; ptV];    clearvars ptH ptV;

% plot a truss skeleton
if DEBUG
    pt_X     = repmat(iniLocW',4,1);
    pt_Y     = reshape(repmat(iniLocH,7,1),28,1);
    
    figure(2);
    line(pt_X(ptMap'),pt_Y(ptMap'),'color','b', 'linewidth', 3);hold on;
    plot(pt_X, pt_Y,'or','linewidth',3);
    text(pt_X+15, pt_Y+45,arrayfun(@num2str, 1:28, 'unif', 0), ...
        'FontSize',14); hold off;
    clearvars ii point_X point_Y;
end

% set threshold values
threshTarget        = 0.1;
targetAreaLB        = 100;
targetAreaUB        = 500;
targetLargeMovement = 100;
circThrsh           = 0.8;

% morphology
se = strel('disk',3);

% video frame rate
camerafs    = 20;

% video file name
filename_video = fullfile(folderOut,'vibration.mp4');

%% Target detection and video creation

% assign global variabales for repetitive gray and BW images (optional)
global img;         
global imgBW;

img     = zeros(imgSize(2),imgSize(1));
imgBW   = zeros(imgSize(2),imgSize(1));

% locations of targets in all images
LocPt   = zeros(28,2,nImg); 

% video writing setup
outputVideo = VideoWriter(filename_video, 'MPEG-4');
outputVideo.FrameRate = camerafs;
open(outputVideo)

fig = figure('visible','off');
for imgIdx = 1: nImg  % image number
    
    img     = rgb2gray(imread(imgPath{imgIdx}));  % load image
    imgBW   = ~(im2bw(img,threshTarget));         % binarizing image
    
% We can reasonably assume a height (vertical) range of targets for their
% detection because targets are only moving in a horizontal direction.
% However, horizontal locations of targets are computed using their
% previous locations. 
    for rowIdx = 1:4      
        % set a range of target in height 
        targetH = iniLocH(rowIdx)-ceil(targetStripH/2)+1: ...
                  iniLocH(rowIdx)+ceil(targetStripH/2);
        if targetH(1) < 1; targetH(1)=1; end;
        if targetH(2) > imgSize(2); targetH(2) = imgSize(2);end;

        % detection of target blobs
        imgTempBW   = imgBW(targetH,1:end); % a row strip
        if DEBUG
            figure(3);
            subplot(211); imshow(uint8(imgTempBW)*255);
            xlabel('B&W image','fontweight','bold');
        end
        imgTempBW = imclearborder(imgTempBW, 8);
        imgTempBW = imclose(imgTempBW,se);
        imgTempBW = xor(bwareaopen(imgTempBW,targetAreaLB), ...
                        bwareaopen(imgTempBW,targetAreaUB));
        if DEBUG
           subplot(212); imshow(uint8(imgTempBW)*255);
           xlabel('B&W image after thresholding','fontweight','bold');
        end
                
        stats     = regionprops(imgTempBW,'Centroid','Perimeter','Area');
        
        % center loc. of a detected blob
        center = bsxfun(@plus, cell2mat({stats.Centroid}'), ...
                        [0 targetH(1)-1]);

        % how much area is a detected blob
        areas = [stats.Area]';
        
        % how much circular is the shape of a detected blob
        perimeter   = [stats.Perimeter]';
        circularity = bsxfun(@(A,B) A.^2./(4*pi*B),perimeter, areas); 
        clearvars perimeter;
        
        % y locations
        yrange = round(bsxfun(@minus,center(:,2), iniLocH(rowIdx)));
        
        % assign locations to LocPt
        if size(center, 1) < 7
            if k>1 % use the previously estimated location
                LocPt(1+(rowIdx-1)*7:(rowIdx)*7,:,imgIdx) ...
                    = LocPt(1+(rowIdx-1)*7:(rowIdx)*7,:,imgIdx-1);
            else % fail to detect blobs at the first image
                LocPt(1+(rowIdx-1)*7:(rowIdx)*7,:,1) = ...
                    [iniLocW' iniLocH(rowIdx)*ones(7,1)]; 
            end
        else
            
            % Condition 1: circularity and yrange variation
            % in other word, find almost circle blob with close to original
            % y location
            % circularity should be close to 1
            indR =  (abs(circularity-1)< 1); 
            
            % y locations should not be more than 5 pixels from the
            % original y location
            indC =  (abs(yrange) < 30);      
            ind  =  and(indR , indC);  % satisfy the both conditions
            ind  =  find(ind == 1); clearvars indR indC;
            
            % Condition 2: Large area is better (Removing noise blobs)
            [~,ind1]  =  sort(areas(ind),'descend');
            
            % best satisfying conditions 1 and 2
            center = center(ind(ind1),:);
            
            % pick first seven points
            center = center(1:7,:);
            [~, indX] = sort(center(:,1),'ascend'); % for numbering
            
            LocPt(1+(rowIdx-1)*7:(rowIdx)*7,:,imgIdx) = center(indX,:);
            
            % One last condition is that pixel movement should not be more
            % than "targetLargeMovement" pixels because those are typically
            % errorneous estimation. if more than "targetLargeMovement"
            % pixels, use previously estimated value.
            if imgIdx>1 && ...
                    any(abs(LocPt(1+(rowIdx-1)*7:(rowIdx)*7,1,imgIdx) ...
                    - LocPt(1+(rowIdx-1)*7:(rowIdx)*7,1,imgIdx-1)) ...
                    > targetLargeMovement);
                LocPt(1+(rowIdx-1)*7:(rowIdx)*7,:,imgIdx) = ...
                    LocPt(1+(rowIdx-1)*7:(rowIdx)*7,:,imgIdx-1);
            elseif imgIdx==1 && ...
                    any(abs(LocPt(1+(rowIdx-1)*7:(rowIdx)*7,1,imgIdx) - ...
                    iniLocW' ) > targetLargeMovement);
                LocPt(1+(rowIdx-1)*7:(rowIdx)*7,:,1) = ...
                    [iniLocW' iniLocH(rowIdx)*ones(7,1)];
            end
        end
    end
    
    % Ploting code (zbuffer_cdata is used for converting fig to image)
    LocPlot(:,1) = LocPt(:,1,imgIdx);
    LocPlot(:,2) = LocPt(:,2,imgIdx);
    
    % 221: test image -----------------------------------------------------
    subplot(4,3,[1 4 7 10]); subimage(img); hold on; % test image
    plot(LocPlot(:,1),LocPlot(:,2), 'og', 'MarkerSize', 2, 'lineWidth',2);
    xlabel('\bf Video','FontSize',12, 'fontweight','bold');
    hold off;    axis off;
    
    % 223: stick figure ---------------------------------------------------
    subplot(4,3, [2 5 8 11]);                % stick figure;
    stickfig = zeros(imgSize(2),imgSize(1));
    subimage(stickfig); hold on;
    line([LocPlot(ptMap(:,1),1),LocPlot(ptMap(:,2),1)]', ...
         [LocPlot(ptMap(:,1),2),LocPlot(ptMap(:,2),2)]','color','b', ...
         'linewidth', 3); hold on;
    plot(LocPlot(:,1),LocPlot(:,2), 'og', 'MarkerSize', 3, 'lineWidth',3);
    xlabel('\bf Skeleton figure','FontSize',12, 'fontweight','bold');
    hold off; axis off;
    
    % 433: stick figure ---------------------------------------------------
    subplot(4,3, [3 6]);
    xdisp =squeeze(LocPt([22:28],1,1:imgIdx));
    plot((1:imgIdx)/camerafs,xdisp-xdisp(:,1)*ones(1,imgIdx),...
        'lineWidth',2);
    ylabel('\bf X disp. at the left colum (px)','FontSize',12, ...
           'fontweight','bold');
    set(gca,'fontsize',12,'linewidth',2,'fontweight','bold'); grid on;
    
    subplot(4,3, [9 12]);
    xdisp = squeeze(LocPt([1 8 15 22],1,1:imgIdx));
    plot((1:imgIdx)/camerafs,xdisp-xdisp(:,1)*ones(1,imgIdx), ...
        'lineWidth',2);
    xlabel('\bf Time(s)','FontSize',12, 'fontweight','bold');
    ylabel('\bf X disp. at the top floor (px)','FontSize',12, ...
           'fontweight','bold');
    set(gca,'fontsize',12,'linewidth',2,'fontweight','bold'); grid on;
    
    set(fig, 'pos',[20 50 1500 600]);hold off; 
    
    writeVideo(outputVideo,getframe(fig));
    
    fprintf('Processing (%d / %d)\n', imgIdx, nImg);
end
disp('Complete!!');
close(outputVideo);