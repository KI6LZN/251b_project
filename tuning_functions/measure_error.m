% Measure frame-wise centroid changes on tracks that are stationary.

% In UA-DETRAC sequence 40851, tracks 2-17 are still for frames 1-251.

% For the vehicles that are sitting still, this function calculates the 
% average Euclidean distance between the centroids of the ground truth 
% bounding boxes between consecutive frames.

function measure_error
    %% Prepare workspace
    close all;
    close(findall(0,'type','figure'));
    clear;
    clc;
    figureCounter = 1;

    num_detections = 16;

    %% Change this number to change the image sequence
    seq_num = '40851';

    %% Environmental Variables
    data_path = insertAfter('detrac/test_images/Insight-MVT_Annotation_Test/MVI_/', 'MVI_', seq_num);
    gndtruth_path = 'detrac/annotations/DETRAC-Test-Annotations-MAT/';
    
    %load ground truth bounding boxes
    anno = open(fullfile(gndtruth_path, insertAfter('MVI_.mat', 'MVI_', seq_num)));
    X = anno.gtInfo.X;
    Y = anno.gtInfo.Y;
    W = anno.gtInfo.W;
    H = anno.gtInfo.H;

    %% Setup file names
    num_frames = 251; 
    imds = imageDatastore(fullfile(data_path),'FileExtensions','.jpg');
    sequence = readall(imds);
    I = sequence{1};
    [height, width, ~] = size(I);
    sequence_filt = zeros([size(I) num_frames],class(I));
    
    %% Read in ground truth values
    % gnd_truth is a 3-D matrix of bounding boxes
    % each k is each frame, and all rows are individual bounding boxes
    gnd_truth_bbox = zeros(num_detections,4,num_frames);
    gnd_truth_centroid = zeros(num_detections,2,num_frames);
    for k = 1:num_frames
        for j = 2:num_detections+1
            w = W(k,j);
            h = H(k,j);
            x = X(k,j);
            y = Y(k,j);
            gnd_truth_centroid(j-1,:,k) = [x,y];
            x = x - floor(w/2);
            y = y - h;
            gnd_truth_bbox(j-1, :, k) = [x,y,w,h];
        end
    end
    
    %% Calculate error between consecutive frames
    meas_error = zeros(num_detections,num_frames-1);
    prev = gnd_truth_centroid(:,:,1);
    for iter = 2:num_frames
        curr = gnd_truth_centroid(:,:,iter);
        meas_error(:,iter-1) = euclidean_distance(curr, prev);        
        prev = curr;
    end
    
    %% Annotate with bounding boxes
    for p = 1:num_frames
        bboxes = gnd_truth_bbox(:,:,p);
        labels = num2str([1:num_detections].');
        
        labels = cellstr(num2str(labels));
        if size(bboxes,1) >= 1
            sequence_filt(:,:,:,p) = insertObjectAnnotation(sequence{p}, ...
                'Rectangle', bboxes, labels);
        else
           sequence_filt(:,:,:,p) = sequence{p}; 
        end
    end
    
    %% Calculate average errors
    col_avg_meas_err = mean(meas_error,1).'; %column vector of average errors across frames
    row_avg_meas_error = mean(meas_error,2); %column vector of average errors across tracks
    avg_row_meas_error = mean(row_avg_meas_error); %single value
    avg_col_meas_error = mean(col_avg_meas_err); %single value
    
    %% Plot average error across frames
    figure(figureCounter)
    figureCounter = figureCounter + 1;
    plot(col_avg_meas_err);
    ylabel('Pixels')
    xlabel('Frame Number')
    title('Average Error Over Each Track in Pixels')
    ylim([0,3])
   
    
    %% Play back images with detected bounding boxes
    m1 = implay(sequence_filt);
    [height, width, ~] = size(I);
    set(findall(0,'tag','spcui_scope_framework'),'position',[150 150 width-100 height-50]);
    play(m1.DataSource.Controls);
end

%% Function to calculate Euclidean distance between two points
function dist = euclidean_distance(A, B)
    dist = sqrt((A(:,1)-B(:,1)).^2 + (A(:,2)-B(:,2)).^2 );
end