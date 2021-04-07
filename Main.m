%% Read the Input Image Sequence
clear variables;
clc;
imageDir = fullfile('frames3/');
imds = imageDatastore(imageDir);

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = rgb2gray(I);
%     imshow(I)
end

%% Load Camera Parameters
load intrinsics.mat ;
% intrinsics=intrinsics1;
writerObj = VideoWriter('example3.avi');
writerObj.FrameRate = 60;
open(writerObj);
%% Create a View Set Containing the First View

% Undistort the first image.
I = undistortImage(images{1}, intrinsics);    

roi = [50, 50, size(I, 2)- 2*50, size(I, 1)- 2*50];
prevPoints   = detectORBFeatures(I,'ROI',roi','ScaleFactor', 1.2, 'NumLevels', 8);
% points = selectUniform(prevPoints, 1000, size(I, 1:2));
prevFeatures = extractFeatures(I, prevPoints);


% imshow(I);
% title('100 Strongest Feature Points from Box Image');
% hold on;
% plot(selectStrongest(prevPoints, 100));
% Create an empty viewSet object to manage the data associated with each
% view.

vSet = imageviewset();

viewId = 1;
vSet = addView(vSet, viewId, rigid3d, 'Points', prevPoints);
%% Add the Rest of the Views
xyz=[];
for i = 2:numel(images)
    
    % Undistort the current image.
    I = undistortImage(images{i}, intrinsics);
    
    % Detect, extract and match features.
    currPoints   = detectORBFeatures(I,'ROI',roi,'ScaleFactor', 1.2, 'NumLevels', 8);
%     currPoints = selectUniform(currPoints, 1000, size(I, 1:2));
    currFeatures = extractFeatures(I, currPoints);
    indexPairs = matchFeatures(prevFeatures, currFeatures,'MaxRatio', .7, 'Unique',  true);

   
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
   
   try

    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, intrinsics);
     
    prevPose = poses(vSet, i-1).AbsolutePose;
    relPose  = rigid3d(relativeOrient, relativeLoc);
        
    currPose = rigid3d(relPose.T * prevPose.T);
    
    vSet = addView(vSet, i, currPose, 'Points', currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, relPose, 'Matches', indexPairs(inlierIdx,:));
    
    tracks = findTracks(vSet);
    camPoses = poses(vSet);
    
    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, intrinsics, ...
        'PointsUndistorted', true);
  
    
    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);
    
    goodIdx = (reprojectionErrors < 1);
    xyzPoints = xyzPoints(goodIdx, :);
   

    camPoses = poses(vSet);
    subplot(1,3,1)
    imshow(I);
    subplot(1,3,2)
    imshow(I);
    hold on;
    plot(currPoints(indexPairs(:, 2)));
    hold off
    subplot(1,3,3)
    XLim = [-100 100];
    YLim = [-100 100];
    ZLim = [0 300];
    plotCamera(camPoses, 'Size', 0.2);
    hold on
    inPlotRange = xyzPoints(:, 1) > XLim(1) & ...
                xyzPoints(:, 1) < XLim(2) & xyzPoints(:, 2) > YLim(1) & ...
                xyzPoints(:, 2) < YLim(2) & xyzPoints(:, 3) > ZLim(1) & ...
                xyzPoints(:, 3) < ZLim(2);
            
    
    pcshow(xyzPoints(inPlotRange, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
        'MarkerSize', 45);
    grid on
    drawnow;
    title('Point cloud');
    hold off
    frame = getframe(gcf);
    writeVideo(writerObj,frame);
    
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
   catch
       disp("Skipping frames as unable to compute essential matrix")
   end
end
close(writerObj);
%%
    camPoses = poses(vSet);
    
    subplot(1,3,2)
     imshow(I);
    subplot(1,3,2)
    imshow(I);
     hold on;
     plot(currPoints(indexPairs(:, 2)));
     hold off
    subplot(1,3,3)
    XLim = [-100 100];
    YLim = [-100 100];
    ZLim = [0 300];
    plotCamera(camPoses, 'Size', 0.2);
    hold on
    inPlotRange = xyzPoints(:, 1) > XLim(1) & ...
                xyzPoints(:, 1) < XLim(2) & xyzPoints(:, 2) > YLim(1) & ...
                xyzPoints(:, 2) < YLim(2) & xyzPoints(:, 3) > ZLim(1) & ...
                xyzPoints(:, 3) < ZLim(2);
            
    xyz=[xyz;xyzPoints(inPlotRange, :)];
    
    pcshow(xyz, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
        'MarkerSize', 45);
    drawnow;
    grid on
    
    title('Sparse point cloud');
    hold off
%     frame = getframe(gcf);
%     writeVideo(writerObj,frame);
%     

%% References
% [1] M.I.A. Lourakis and A.A. Argyros (2009). "SBA: A Software Package for 
% Generic Sparse Bundle Adjustment". ACM Transactions on Mathematical Software 
% (ACM) 36 (1): 1-30.
% 
% [2] R. Hartley, A. Zisserman, "Multiple View Geometry in Computer Vision," 
% Cambridge University Press, 2003.
% 
% [3] B. Triggs; P. McLauchlan; R. Hartley; A. Fitzgibbon (1999). "Bundle Adjustment: 
% A Modern Synthesis". Proceedings of the International Workshop on Vision Algorithms. 
% Springer-Verlag. pp. 298-372.
% 
% _Copyright 2015 The MathWorks, Inc._