% Display picture sequence as a video

%% Prepare workspace
close all;
close(findall(0,'type','figure'));
clear;
clc;

%% Change this number to change the image sequence
seq_num = '39031';

%% Environmental Variables
data_path = insertAfter('detrac/test_images/Insight-MVT_Annotation_Test/MVI_/', 'MVI_', seq_num);

%% Setup file names
dirOutput = dir(fullfile(data_path,'img*.jpg'));
fileNames = {dirOutput.name}.';
numFrames = numel(fileNames);

%% Read images into MATLAB
I = imread(fullfile(data_path, fileNames{1}));
sequence = zeros([size(I) numFrames],class(I));
sequence(:,:,:,1) = I;

for p = 2:numFrames
    sequence(:,:,:,p) = imread(fullfile(data_path, fileNames{p}));
end

%% Play images as a video
m1 = implay(sequence);
[width, height] = size(I);
set(findall(0,'tag','spcui_scope_framework'),'position',[150 150 width+250 height/6]);
play(m1.DataSource.Controls);