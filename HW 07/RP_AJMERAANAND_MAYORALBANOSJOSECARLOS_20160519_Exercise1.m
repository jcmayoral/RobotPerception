%-------------------------------------------------------------------
% Calibration Images
%-------------------------------------------------------------------

path = '/home/jose/Dropbox/MAS/Semester_2/Robot Perception/HW/HW 07/images5/';

numImages = 12;
files = cell(1, numImages);
for i = 1:numImages
    %RP_MAYORALBAÑOSJOSECARLOS_20160519
    files{i} = fullfile(path, sprintf('image%d.jpg', i));
    %files{i} = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', ...
    %    'calibration', 'slr', sprintf('image%d.jpg', i));   
    %disp(files{i})
end

% Display one of the calibration images
%magnification = 25;
%figure; imshow(files{1}, 'InitialMagnification', magnification);
%title('One of the Calibration Images');

%-------------------------------------------------------------------
% Estimate Camera Parameters
%-------------------------------------------------------------------

% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 24; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
cameraParams = estimateCameraParameters(imagePoints, worldPoints);
disp(cameraParams)

% Evaluate calibration accuracy.
figure; showReprojectionErrors(cameraParams);
title('Reprojection Errors');

%-------------------------------------------------------------------
% Estimate Camera Parameters
%-------------------------------------------------------------------
magnification = 25;
imOrig = imread(fullfile(path,'image1.jpg'));
%figure; imshow(imOrig, 'InitialMagnification', magnification);
%title('Input Image');

%-------------------------------------------------------------------
% Undistorted Image
%-------------------------------------------------------------------

[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
figure; imshow(im, 'InitialMagnification', magnification);
title('Undistorted Image');


%-------------------------------------------------------------------
% Segmented Coins
%-------------------------------------------------------------------

% Convert the image to the HSV color space.
imHSV = rgb2hsv(im);

% Get the saturation channel.
saturation = imHSV(:, :, 2);

% Threshold the image
t = graythresh(saturation);
imCoin = (saturation > t);

figure; imshow(imCoin, 'InitialMagnification', magnification);
title('Segmented Coins');

%-------------------------------------------------------------------
% Detected Coins
%-------------------------------------------------------------------

% Find connected components.
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true);
[areas, boxes] = step(blobAnalysis, imCoin);

% Sort connected components in descending order by area
[~, idx] = sort(areas, 'Descend');

% Get the two largest components.
boxes = double(boxes(idx(1:2), :));

% Adjust for coordinate system shift caused by undistortImage
boxes(:, 1:2) = bsxfun(@plus, boxes(:, 1:2), newOrigin);

% Reduce the size of the image for display.
scale = magnification / 100;
imDetectedCoins = imresize(im, scale);

% Insert labels for the coins.
imDetectedCoins = insertObjectAnnotation(imDetectedCoins, 'rectangle', ...
    scale * boxes, 'penny');
figure; imshow(imDetectedCoins);
title('Detected Coins');

%-------------------------------------------------------------------
% Compute Extrinsic
%-------------------------------------------------------------------

% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);



%-------------------------------------------------------------------
% Measure first coin
%-------------------------------------------------------------------

% Get the top-left and the top-right corners.
box1 = double(boxes(1, :));
imagePoints1 = [box1(1:2); ...
                box1(1) + box1(3), box1(2)];

% Get the world coordinates of the corners
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

% Compute the diameter of the coin in millimeters.
d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
fprintf('Measured diameter of one penny = %0.2f mm\n', diameterInMillimeters);


%-------------------------------------------------------------------
% Measure second coin
%-------------------------------------------------------------------

% Get the top-left and the top-right corners.
box2 = double(boxes(2, :));
imagePoints2 = [box2(1:2); ...
                box2(1) + box2(3), box2(2)];

% Apply the inverse transformation from image to world
worldPoints2 = pointsToWorld(cameraParams, R, t, imagePoints2);

% Compute the diameter of the coin in millimeters.
d = worldPoints2(2, :) - worldPoints2(1, :);
diameterInMillimeters = hypot(d(1), d(2));
fprintf('Measured diameter of the other penny = %0.2f mm\n', diameterInMillimeters);


%-------------------------------------------------------------------
% Measure distance to first coin
%-------------------------------------------------------------------

% Compute the center of the first coin in the image.
center1_image = box1(1:2) + box1(3:4)/2;

% Convert to world coordinates.
center1_world  = pointsToWorld(cameraParams, R, t, center1_image);

% Remember to add the 0 z-coordinate.
center1_world = [center1_world 0];

% Compute the distance to the camera.
distanceToCamera = norm(center1_world + t);
fprintf('Distance from the camera to the first penny = %0.2f mm\n', ...
    distanceToCamera);