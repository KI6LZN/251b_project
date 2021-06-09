% Track all vehicles across the frames with a choice of filters, each with
% the "Constant Turn" motion model

% Note that the Kalman filter will still use the "Constant Velocity" motion
% model

% Portions of the code are modified versions of MATLAB's 
% "MotionBasedMultiObjectTrackingExample" code, available:
% https://www.mathworks.com/help/vision/ug/motion-based-multiple-object-tracking.html

function track_function_const_turn
    %% Prepare workspace
    close all;
    close(findall(0,'type','figure'));
    clear;
    clc;
    figureCounter = 1;
    
    rng(3);
    
    %% Select which filter to use
    valid_filt_names = ["Kalman", "UKF", "PF"];
    filt_name = valid_filt_names(3);

    %% Change this number to change the image sequence
    seq_num = '39031'; %'40701'; %'39031'; %'40851';

    %% Environmental Variables
    data_path = insertAfter('detrac/test_images/Insight-MVT_Annotation_Test/MVI_/', 'MVI_', seq_num);
    gndtruth_path = 'detrac/annotations/DETRAC-Test-Annotations-MAT/';
    
    %% Read in ground truth values
    anno = open(fullfile(gndtruth_path, insertAfter('MVI_.mat', 'MVI_', seq_num)));
    X = anno.gtInfo.X;
    Y = anno.gtInfo.Y;
    W = anno.gtInfo.W;
    H = anno.gtInfo.H;
    num_detections = size(X,2);
    
    %% Read images into MATLAB
    imds = imageDatastore(fullfile(data_path),'FileExtensions','.jpg');
    sequence = readall(imds);
    num_frames = length(sequence);
    I = sequence{1};
    [height, width, ~] = size(I);
    
    
    %% Prepare ground truth bounding boxes
    % gnd_truth is a 3-D matrix of bounding boxes
    % each k is each frame, and all rows are individual bounding boxes
    gnd_truth_bbox = zeros(num_detections,4,num_frames);
    gnd_truth_centroid = zeros(num_detections,2,num_frames);
    for k = 1:num_frames
        for j = 1:num_detections
            w = W(k,j);
            h = H(k,j);
            x = X(k,j);
            y = Y(k,j);
            gnd_truth_centroid(j,:,k) = [x,y];
            x = x - floor(w/2);
            y = y - h;
            gnd_truth_bbox(j, :, k) = [x,y,w,h];
        end
    end
    
    
    %% Detect and track vehicles
    centroid_log = [];
    pair_log = {};
    dist_log = {};
    mean_dist = zeros(num_frames,1);
    tracks = initializeTracks;
    nextId = 1; % ID of the next track
    tic %timing
    for p = 1:num_frames
        frame_cents = [];

        % load ground truth values for this frame
        bbox = gnd_truth_bbox(:,:,p);
        centroids = gnd_truth_centroid(:,:,p);
        
        % generate a log variable with the track number and centroid
        % location for each ground truth centroid in each frame
        cent_with_track = [];
        for j = 1:size(centroids,1)
            if ~(centroids(j,1) == 0 && centroids(j,2) == 0)
                cent_with_track = [cent_with_track; j centroids(j,1) centroids(j,2)];
            end
        end

        % generate a log variable with the track number and centroid
        % location for each predicted centroid in each frame
        tracks = predictNewLocationsOfTracks(tracks, filt_name);
        for j = 1:length(tracks)
            filt = getfield(tracks(j), filt_name);
            st = filt.State;
            frame_cents = [frame_cents; tracks(j).id st(1) st(3)];
        end
        centroid_log = [centroid_log; repmat(p,size(frame_cents,1), 1) frame_cents ];
        
        % update track assignments and remove tracks that have disappeared
        [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment(tracks, centroids, filt_name);
        tracks = updateAssignedTracks(tracks, assignments, centroids, bbox, filt_name);
        tracks = updateUnassignedTracks(tracks, unassignedTracks);
        tracks = deleteLostTracks(tracks);
        [tracks, nextId] = createNewTracks(tracks, centroids, ...
            bbox, unassignedDetections, nextId);
        
        
        
%         for j = 1:length(tracks)
%             filt = getfield(tracks(j), filt_name);
%             st = filt.State;
%             frame_cents = [frame_cents; tracks(j).id st(1) st(3)];
%         end
%         centroid_log = [centroid_log; repmat(p,size(frame_cents,1), 1) frame_cents ];
        
        %% prepare bboxes and track number labels
        labels = [];
        bboxes = [];
        for i = 1:length(tracks)
            if tracks(i).totalVisibleCount >= 8
               bboxes = [bboxes; tracks(i).bbox];
               labels = [labels; tracks(i).id];
            end
            
        end
        
        %% add bboxes
        labels = cellstr(num2str(labels));
        if size(bboxes,1) >= 1
            sequence_filt(:,:,:,p) = insertObjectAnnotation(sequence{p}, ...
                'Rectangle', bboxes, labels);
        else
           sequence_filt(:,:,:,p) = sequence{p}; 
        end
        
        %% logging and comparison
        [pairs, dist] = associate_tracks(cent_with_track, frame_cents);
        dist_log{p} = dist;
        pair_log{p} = pairs;
        mean_dist(p) = mean(dist);
    end
    toc %timing
    
    TF = isnan( mean_dist );
    min_dist = min(mean_dist(mean_dist>0))
    max_dist = max(mean_dist)
    omean_dist = mean(mean_dist(~TF))
    
    
    %% Plot average error
    figure(figureCounter)
    figureCounter = figureCounter + 1;
    plot(mean_dist)
    title_str = strcat("Average Centroid Error in Pixels, Constant Turn: ", filt_name);
    sub_str = strcat("Seq. ", num2str(seq_num));
    title(title_str, sub_str)
    ylabel("Pixels")
    xlabel("Frame")
    
    
    %% Play back images with detected bounding boxes
    m1 = implay(sequence_filt);
    [height, width, ~] = size(I);
    set(findall(0,'tag','spcui_scope_framework'),'position',[150 150 width-100 height-50]);
    play(m1.DataSource.Controls);

end

%% track structure, modified from MATLAB's "MotionBasedMultiObjectTrackingExample"
function tracks = initializeTracks()
    % create an empty array of tracks
    tracks = struct(...
        'id', {}, ...
        'bbox', {}, ...
        'Kalman', {}, ...
        'UKF', {}, ...
        'PF', {}, ...
        'age', {}, ...
        'totalVisibleCount', {}, ...
        'consecutiveInvisibleCount', {});
end

%% associate track numbers with currently visible centroids
%pairs is a column vector where the row number is the track number of the
%true centroid, and the value is the track number of the frame centroid
function [pairs, dist] = associate_tracks(true_cent, frame_cents)
    
    if isempty(frame_cents) || isempty(true_cent)
        pairs = [];
        dist = 0;
        return;
    end

    n_true = size(true_cent,1);
    nDetections = size(frame_cents,1);
    cost = zeros(n_true, nDetections);
    
    frame_cent = frame_cents(:,[2 3]);
    gnd_cent = true_cent(:,[2 3]);
    
    %for all ground truth tracks
    for iter = 1:n_true
        true_val = repmat(gnd_cent(iter,:), nDetections);
        cost(iter,:) = euclidean_distance(true_val, frame_cent).';
    end
       
    % Solve the assignment problem.
    costOfNonAssignment = 20;
    [pairs, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(cost, costOfNonAssignment);
    
    idx = sub2ind(size(cost), pairs(:,1), pairs(:,2));
    dist = cost(idx);
    
    for iter = 1:size(pairs,1)
       pairs(iter, 1) = true_cent(iter,1);
       int_track = pairs(iter, 2);
       pairs(iter, 2) = frame_cents(int_track,1);
    end
end

%% Calculate Euclidean distance between two (x,y) column vectors
function dist = euclidean_distance(A, B)
    dist = sqrt((A(:,1)-B(:,1)).^2 + (A(:,2)-B(:,2)).^2 );
end

%% Modified version of function from MATLAB's "MotionBasedMultiObjectTrackingExample"
function tracks = predictNewLocationsOfTracks(tracks, filt_name)
    for i = 1:length(tracks)
        % Predict the current location of the track.
        bbox = tracks(i).bbox;
        if filt_name == "Kalman"
            predictedCentroid = predict(tracks(i).Kalman);
        elseif filt_name == "UKF"
            [predictedCentroid, ~] = predict(tracks(i).UKF);
            predictedCentroid = [predictedCentroid(1), predictedCentroid(3)];
        elseif filt_name == "PF"
            [predictedCentroid, ~] = predict(tracks(i).PF);
            predictedCentroid = [predictedCentroid(1), predictedCentroid(3)];            
        end       
        

        % Shift the bounding box so that its center is at 
        % the predicted location.
        predictedCentroid = int32(predictedCentroid) - int32(bbox(3:4)) / 2;
        tracks(i).bbox = [predictedCentroid, bbox(3:4)];
    end
end

%% Modified version of function from MATLAB's "MotionBasedMultiObjectTrackingExample"
function [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment(tracks, centroids, filt_name)

    nTracks = length(tracks);
    nDetections = size(centroids, 1);

    % Compute the cost of assigning each detection to each track.
    cost = zeros(nTracks, nDetections);
    for i = 1:nTracks
        if filt_name == "Kalman"
            cost(i, :) = distance(tracks(i).Kalman, centroids);
        elseif filt_name == "UKF"
            meas = [centroids, zeros(size(centroids,1),1)];
            if ~isempty(meas)
                cost(i, :) = distance(tracks(i).UKF, meas);
            else
                cost(i, :) = zeros(1, size(centroids,1));
            end
        elseif filt_name == "PF"
            meas = [centroids, zeros(size(centroids,1),1)];
            if ~isempty(meas)
                cost(i, :) = distance(tracks(i).PF, meas);
            else
                cost(i, :) = zeros(1, size(centroids,1));
            end
        end
    end

    % Solve the assignment problem.
    costOfNonAssignment = 20;
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(cost, costOfNonAssignment);
    
end

%% Modified version of function from MATLAB's "MotionBasedMultiObjectTrackingExample"
function tracks = updateAssignedTracks(tracks, assignments, centroids, bboxes, filt_name)
    numAssignedTracks = size(assignments, 1);
    for i = 1:numAssignedTracks
        trackIdx = assignments(i, 1);
        detectionIdx = assignments(i, 2);
        centroid = centroids(detectionIdx, :);
        bbox = bboxes(detectionIdx, :);

        % Correct the estimate of the object's location
        % using the new detection.
        if filt_name == "Kalman"
            correct(tracks(trackIdx).Kalman, centroid);
        elseif filt_name == "UKF"
            centroid = [centroid, 0];
            correct(tracks(trackIdx).UKF, centroid);
        elseif filt_name == "PF"
            centroid = [centroid, 0];
            correct(tracks(trackIdx).PF, centroid);
        end

        % Replace predicted bounding box with detected
        % bounding box.
        tracks(trackIdx).bbox = bbox;

        % Update track's age.
        tracks(trackIdx).age = tracks(trackIdx).age + 1;

        % Update visibility.
        tracks(trackIdx).totalVisibleCount = ...
            tracks(trackIdx).totalVisibleCount + 1;
        tracks(trackIdx).consecutiveInvisibleCount = 0;
    end
end

%% Function from MATLAB's "MotionBasedMultiObjectTrackingExample"
function tracks = updateUnassignedTracks(tracks, unassignedTracks)
    for i = 1:length(unassignedTracks)
        ind = unassignedTracks(i);
        tracks(ind).age = tracks(ind).age + 1;
        tracks(ind).consecutiveInvisibleCount = ...
            tracks(ind).consecutiveInvisibleCount + 1;
    end
end

%% Function from MATLAB's "MotionBasedMultiObjectTrackingExample"
function tracks = deleteLostTracks(tracks)
    if isempty(tracks)
        return;
    end

    invisibleForTooLong = 10;
    ageThreshold = 8;

    % Compute the fraction of the track's age for which it was visible.
    ages = [tracks(:).age];
    totalVisibleCounts = [tracks(:).totalVisibleCount];
    visibility = totalVisibleCounts ./ ages;

    % Find the indices of 'lost' tracks.
    lostInds = (ages < ageThreshold & visibility < 0.6) | ...
        [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

    % Delete lost tracks.
    tracks = tracks(~lostInds);
end

%% Modified version of function from MATLAB's "MotionBasedMultiObjectTrackingExample"
function [tracks, nextId] = createNewTracks(tracks, centroids, ...
    bboxes, unassignedDetections, nextId)
    centroids = centroids(unassignedDetections, :);
    bboxes = bboxes(unassignedDetections, :);

    for i = 1:size(centroids, 1)

        centroid = centroids(i,:);
        bbox = bboxes(i, :);

        % Create a Kalman filter object.
        kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
            centroid, [0.7826, 0.75], [0.7826, 0.75], 0.7826);
        
        % Create an Unscented Kalman Filter object
        %1.51 and 1.19, 0.7826 for 39031
        %0.7826 and 0.75, meas = 1 for 40701
        %0.875 and 1.275, 0.7826 for 40851
        cov = 1.51.*eye(5); 
        cov(1,1) = 1.19;
        cov(3,3) = 1.19;
        meas = 0.7826;
        ukf = trackingUKF(@constturn,@ctmeas,[centroid(1);0;centroid(2);0;0.1],...
            'StateCovariance', cov, 'ProcessNoise', cov, ...
            'MeasurementNoise', meas, 'Alpha', 1e-2);
        
        % Create a Particle Filter object
        % 39031: 2, 3, 0.001, 3, 10000: 1.1094 (1000: 1.2005) 1,3,0.0005,3: 1.1132
        % 40701: 1, 2, 0.001, 3, 10000: 0.9196 (1000: 0.9728, 500: 1.0055)
        % 40851: 2, 3, 0.0005, 4, 10000: 0.9516 (1000: 1.0037)
        pos = 2; %0.7826;
        vel = 3; %0.75;
        cov = eye(4);
        cov(1,1) = pos;
        cov(2,2) = vel;
        cov(3,3) = pos;
        cov(4,4) = vel;
        cov(5,5) = 0.001;
        meas = 3;
        omega = 0.1;
        num_particles = 10000;
        pf = trackingPF(@constturn,@ctmeas,[centroid(1);0;centroid(2);0;omega],...
        'StateCovariance', cov, 'ProcessNoise', cov, ...
        'MeasurementNoise', meas, 'NumParticles', num_particles, ...
        'ResamplingMethod', 'Systematic');

        % Create a new track.
        newTrack = struct(...
            'id', nextId, ...
            'bbox', bbox, ...
            'Kalman', kalmanFilter, ...
            'UKF', ukf, ...
            'PF', pf, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0);

        % Add it to the array of tracks.
        tracks(end + 1) = newTrack;

        % Increment the next id.
        nextId = nextId + 1;
    end
end