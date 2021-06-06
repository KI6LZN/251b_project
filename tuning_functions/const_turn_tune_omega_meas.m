% Track all vehicles across the frames with a choice of filters, each with
% the "Constant Turn" motion model.

% This file tests the effect that different measurement covariance and omega
% parameters have on the average centroid error across the different filters


% Portions of the code are modified versions of MATLAB's 
% "MotionBasedMultiObjectTrackingExample" code, available:
% https://www.mathworks.com/help/vision/ug/motion-based-multiple-object-tracking.html

function const_turn_tune_omega_meas
    %% Prepare workspace
    close all;
    close(findall(0,'type','figure'));
    clear;
    clc;
    figureCounter = 1;
    
    %% Change this number to change the test covariance values
    omega = 10.^(-6:1:-1);
    omega = [omega, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 10.^(2:1:3)];
    num_omega = size(omega,2);
    
    %plotting variable
    mean_omega = zeros(num_omega, 1);
    num_tracks_omega = zeros(num_omega, 1);
    
    meas = 10.^(-6:1:-1);
    meas = [meas, 0.25, 0.5, 0.7826, 1, 1.25, 1.5, 10.^(2:1:3)];
    num_meas = size(meas,2);
    
    %plotting variable
    mean_meas = zeros(num_meas, 1);
    num_tracks_meas = zeros(num_meas, 1);
    
    %% Select which filter to use
    valid_filt_names = ["Kalman", "UKF", "PF", "RLS"];

    %% Change this number to change the image sequence
    seq_num = '40851'; %'40701'; %'39031'; %'40851';

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
    
    
    %% Run tests
    tic %timing
    %for all filters
    idx = 2;
%     for idx = 1:2
        filt_name = valid_filt_names(idx);
        %for all test velocity covariance values
        for iter = 1:num_omega
            %% Detect and track vehicles
            centroid_log = [];
            pair_log = {};
            dist_log = {};
            mean_dist = zeros(num_frames,1);
            tracks = initializeTracks;
            nextId = 1; % ID of the next track
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
                [tracks, nextId] = createNewTracks_1(tracks, centroids, ...
                    bbox, unassignedDetections, nextId, omega(iter));



        %         for j = 1:length(tracks)
        %             filt = getfield(tracks(j), filt_name);
        %             st = filt.State;
        %             frame_cents = [frame_cents; tracks(j).id st(1) st(3)];
        %         end
        %         centroid_log = [centroid_log; repmat(p,size(frame_cents,1), 1) frame_cents ];


                %% logging and comparison
                [pairs, dist] = associate_tracks(cent_with_track, frame_cents);
                dist_log{p} = dist;
                pair_log{p} = pairs;
                mean_dist(p) = mean(dist);
            end
            mean_omega(iter,idx-1) = mean(mean_dist(:));
            num_tracks_omega(iter,idx-1) = nextId-1;
        end
%     end
    toc %timing
    
        %% Run tests
    tic %timing
    %for all filters
%     for idx = 1:2
        filt_name = valid_filt_names(idx);
        %for all test velocity covariance values
        for iter = 1:num_meas
            %% Detect and track vehicles
            centroid_log = [];
            pair_log = {};
            dist_log = {};
            mean_dist = zeros(num_frames,1);
            tracks = initializeTracks;
            nextId = 1; % ID of the next track
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
                [tracks, nextId] = createNewTracks_2(tracks, centroids, ...
                    bbox, unassignedDetections, nextId, meas(iter));



        %         for j = 1:length(tracks)
        %             filt = getfield(tracks(j), filt_name);
        %             st = filt.State;
        %             frame_cents = [frame_cents; tracks(j).id st(1) st(3)];
        %         end
        %         centroid_log = [centroid_log; repmat(p,size(frame_cents,1), 1) frame_cents ];

                %% logging and comparison
                [pairs, dist] = associate_tracks(cent_with_track, frame_cents);
                dist_log{p} = dist;
                pair_log{p} = pairs;
                mean_dist(p) = mean(dist);
            end
            mean_meas(iter,idx-1) = mean(mean_dist(:));
            num_tracks_meas(iter,idx-1) = nextId-1;
        end
%     end
    toc %timing
    
    %% Plot average error
    figure(figureCounter)
    figureCounter = figureCounter + 1;
    subplot(2,1,1);
    title_str = strcat("Average Centroid Error in Pixels, Constant Turn, Seq. ", num2str(seq_num));
    sub_str = "Velocity Covariance";
    %title(title_str, sub_str)
    semilogx(omega, mean_omega(:,1), 'bo--')
%     legend("Kalman Filter", "UKF")
    ylabel("Pixels")
    ylim([min(mean_omega(:))-0.25,max(mean_omega(:))+0.25])
    xlabel("Omega Value")
    set(gca, 'FontName', 'Times New Roman');

    
    subplot(2,1,2);
    semilogx(meas, mean_meas, 'bo--')
%     legend("Kalman Filter", "UKF")
    title_str = ""; %strcat("Average Centroid Error in Pixels, Constant Velocity");
    sub_str = "Measurement Variance";
    %title(title_str, sub_str)
    ylabel("Pixels")
    ylim([min(mean_meas(:))-0.25,max(mean_meas(:))+0.25])
    xlabel("Measurement Variance")
    set(gca, 'FontName', 'Times New Roman');
    
    subplot(2,1,1);
    title_str = strcat("Average Centroid Error in Pixels, Constant Turn, Seq. ", num2str(seq_num));
    %sub_str = "Velocity Covariance";
    title(title_str)%, sub_str)
    set(gca, 'FontName', 'Times New Roman');
    
    %more tracks is bad - means we didn't successfully associate
    num_tracks_omega
    num_tracks_meas
    

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
        'RLS', {}, ...
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
        elseif filt_name == "RLS"
            
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
        elseif filt_name == "RLS"
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
        elseif filt_name == "RLS"
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
function [tracks, nextId] = createNewTracks_1(tracks, centroids, ...
    bboxes, unassignedDetections, nextId, omega)
    centroids = centroids(unassignedDetections, :);
    bboxes = bboxes(unassignedDetections, :);

    for i = 1:size(centroids, 1)

        centroid = centroids(i,:);
        bbox = bboxes(i, :);

        % Create a Kalman filter object.
        kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
            centroid, [0.7826, 0.75], [0.7826, 0.75], 0.7826);
        
        % Create an Unscented Kalman Filter object
        cov = 1.25.*eye(5);
%         cov_2 = cov;
        cov(2,2) = 0.75;
        cov(4,4) = 0.75;
        meas = 1;
        ukf = trackingUKF(@constturn,@ctmeas,[centroid(1);0;centroid(2);0;omega],...
            'StateCovariance', cov, 'ProcessNoise', cov, ...
            'MeasurementNoise', meas, 'Alpha', 1e-2);
        
        % Create a Particle Filter object
        pf = trackingPF(@constvel,@cvmeas,[centroid(1);0;centroid(2);0], ...
            'NumParticles',2500);

        % Create a new track.
        newTrack = struct(...
            'id', nextId, ...
            'bbox', bbox, ...
            'Kalman', kalmanFilter, ...
            'UKF', ukf, ...
            'PF', pf, ...
            'RLS', 1, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0);

        % Add it to the array of tracks.
        tracks(end + 1) = newTrack;

        % Increment the next id.
        nextId = nextId + 1;
    end
end

%% Modified version of function from MATLAB's "MotionBasedMultiObjectTrackingExample"
function [tracks, nextId] = createNewTracks_2(tracks, centroids, ...
    bboxes, unassignedDetections, nextId, meas)
    centroids = centroids(unassignedDetections, :);
    bboxes = bboxes(unassignedDetections, :);

    for i = 1:size(centroids, 1)

        centroid = centroids(i,:);
        bbox = bboxes(i, :);

        % Create a Kalman filter object.
        kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
            centroid, [meas, 0.75], [meas, 0.75], 0.7826);
        
        % Create an Unscented Kalman Filter object
        cov = 0.75.*eye(5);
        cov(1,1) = 1.25;
        cov(3,3) = 1.25;
%         meas = 0.7826;
        ukf = trackingUKF(@constturn,@ctmeas,[centroid(1);0;centroid(2);0;0.1],...
            'StateCovariance', cov, 'ProcessNoise', cov, ...
            'MeasurementNoise', meas, 'Alpha', 1e-2);
        
        % Create a Particle Filter object
        pf = trackingPF(@constvel,@cvmeas,[centroid(1);0;centroid(2);0], ...
            'NumParticles',2500);

        % Create a new track.
        newTrack = struct(...
            'id', nextId, ...
            'bbox', bbox, ...
            'Kalman', kalmanFilter, ...
            'UKF', ukf, ...
            'PF', pf, ...
            'RLS', 1, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0);

        % Add it to the array of tracks.
        tracks(end + 1) = newTrack;

        % Increment the next id.
        nextId = nextId + 1;
    end
end