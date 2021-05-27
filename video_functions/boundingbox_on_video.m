% Add ground truth bounding boxes onto video

%% Prepare workspace
close all;
close(findall(0,'type','figure'));
clear;
clc;

%% Change this number to change the image sequence
seq_num = '40851';% '40792';

%% Environmental Variables
data_path = insertAfter('detrac/test_images/Insight-MVT_Annotation_Test/MVI_/', 'MVI_', seq_num);
gndtruth_path = 'detrac/annotations/DETRAC-Test-Annotations-MAT/';

%% Load annotations
anno = open(fullfile(gndtruth_path, insertAfter('MVI_.mat', 'MVI_', seq_num)));
X = anno.gtInfo.X;
Y = anno.gtInfo.Y;
W = anno.gtInfo.W;
H = anno.gtInfo.H;

%% Read images into MATLAB
imds = imageDatastore(fullfile(data_path),'FileExtensions','.jpg');
sequence = readall(imds);
I = sequence{1};
num_frames = length(sequence);
num_detections = size(X,2);

%% Read in ground truth values
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

%bbox labels
labels = (1:num_detections).';
labels = cellstr(num2str(labels));

% output variable
seq = zeros([size(I) num_frames],class(I));

%% Insert bounding boxes
for p = 1:num_frames    
    seq(:,:,:,p) = insertObjectAnnotation(sequence{p}, ...
        'Rectangle', gnd_truth_bbox(:,:,p), labels);
end
   
%% Play images as a video
m1 = implay(seq);
[height, width, ~] = size(I);
set(findall(0,'tag','spcui_scope_framework'),'position',[150 150 width-100 height-50]);
play(m1.DataSource.Controls);