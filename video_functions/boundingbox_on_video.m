% Add ground truth bounding boxes onto video

%% Prepare workspace
close all;
close(findall(0,'type','figure'));
clear;
clc;

%% Change this number to change the image sequence
seq_num = '39031';

%% Environmental Variables
data_path = insertAfter('detrac/test_images/Insight-MVT_Annotation_Test/MVI_/', 'MVI_', seq_num);
gndtruth_path = 'detrac/annotations/DETRAC-Test-Annotations-MAT/';

%% Load annotations
anno = open(fullfile(gndtruth_path, insertAfter('MVI_.mat', 'MVI_', seq_num)));
X = anno.gtInfo.X;
Y = anno.gtInfo.Y;
W = anno.gtInfo.W;
H = anno.gtInfo.H;

numDet = size(X,2);

%% Setup file names
dirOutput = dir(fullfile(data_path,'img*.jpg'));
fileNames = {dirOutput.name}.';
numFrames = numel(fileNames);

%% Read images into MATLAB
I = imread(fullfile(data_path, fileNames{1}));
sequence = zeros([size(I) numFrames],class(I));
sequence(:,:,:,1) = I;

%for all detections, if there is a bounding box for the first frame frame
for j = 1:numDet
    if X(1,j) ~= 0
        w = W(1,j);
        h = H(1,j);
        x = X(1,j) - floor(w/2);
        y = Y(1,j) - h;
        sequence(:,:,:,1) = insertShape(sequence(:,:,:,1), 'Rectangle', [x y w h]);
    end
end

%for all frames
for p = 2:numFrames
    sequence(:,:,:,p) = imread(fullfile(data_path, fileNames{p}));
    
    %for all detections, if there is a bounding box for this frame
    for j = 1:numDet
        if X(p,j) ~= 0
            w = W(p,j);
            h = H(p,j);
            x = X(p,j) - floor(w/2);
            y = Y(p,j) - h;
            
            sequence(:,:,:,p) = insertShape(sequence(:,:,:,p), 'Rectangle', [x y w h]);
        end
    end
end

%% Play images as a video
m1 = implay(sequence);
[width, height] = size(I);
set(findall(0,'tag','spcui_scope_framework'),'position',[150 150 width+250 height/6]);
play(m1.DataSource.Controls);