% Display picture sequence as a video

%% Prepare workspace
close all;
close(findall(0,'type','figure'));
clear;
clc;

%% Change this number to change the image sequence
seq_num = '40864';

%% Environmental Variables
data_path = insertAfter('detrac/test_images/Insight-MVT_Annotation_Test/MVI_/', 'MVI_', seq_num);

%% Read images into MATLAB
imds = imageDatastore(fullfile(data_path),'FileExtensions','.jpg');
sequence = readall(imds);
I = sequence{1};
num_frames = length(sequence);

%% Read sequence into array instead of cells
play_seq = zeros([size(I) num_frames],class(I));
for p = 1:num_frames    
    play_seq(:,:,:,p) = sequence{p};
end

%% Play images as a video
m1 = implay(play_seq);
[width, height] = size(I);
set(findall(0,'tag','spcui_scope_framework'),'position',[150 150 width+250 height/6]);
play(m1.DataSource.Controls);