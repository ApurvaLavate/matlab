clc; clear; close all;

% Load video
video = VideoReader('C:\Users\apurv\Downloads\parking1.mp4'); % Replace with your video file

% Create Foreground Detector (Gaussian Mixture Model)
foregroundDetector = vision.ForegroundDetector('NumGaussians', 3, 'LearningRate', 0.01);

% Initialize Kalman Filter for Constant Velocity Model
kalmanFilter = configureKalmanFilter('ConstantVelocity', [0, 0], [1 1], [1 1], 1);

% Create a figure window
figure;

while hasFrame(video)
    frame = readFrame(video); % Read current frame

    % Detect moving objects (Foreground Mask)
    mask = step(foregroundDetector, frame);

    % Morphological processing to remove noise
    mask = imopen(mask, strel('disk', 3)); % Remove small noise
    mask = imclose(mask, strel('disk', 7)); % Fill small gaps
    mask = bwareaopen(mask, 150); % Remove small regions

    % Find object properties
    stats = regionprops(mask, 'Centroid', 'BoundingBox');

    detectedPosition = []; % Initialize detected position

    if ~isempty(stats)
        % Select the largest detected moving object (assumed to be the car)
        [~, maxIndex] = max(cellfun(@(x) prod(x), {stats.BoundingBox}));
        detectedPosition = stats(maxIndex).Centroid;
        boundingBox = stats(maxIndex).BoundingBox;
    end

    % Display results
    imshow(frame); hold on;

    % Draw bounding box around detected car
    if ~isempty(detectedPosition)
        rectangle('Position', boundingBox, 'EdgeColor', 'g', 'LineWidth', 2);
        plot(detectedPosition(1), detectedPosition(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    end

    legend({'Detected Position'});

    hold off;
    pause(0.02); % Small delay for visualization
end

disp('Car Tracking Complete.');
