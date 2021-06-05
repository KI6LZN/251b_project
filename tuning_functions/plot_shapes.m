% function to plot the shapes used for the ablation tests

function plot_shapes
    %% Prepare workspace
    close all;
    close(findall(0,'type','figure'));
    clear;
    clc;
    figureCounter = 1;
    
    %% generate a line with a slope of 1
    line_x = 0:.5:10;
    line_y = line_x;
    line_x = [line_x, 10.5:.5:20]./20;
    line_y = [line_y, 11:2:50]./20;
    
    %% generate a circle with a radius of 1
    theta = 0:pi/25:2*pi;
    r = 1;
    circle_x = r.*cos(theta);
    circle_y = r.*sin(theta);
    
    
    %% generate a sinusoid with an amplitude of 1
    theta = 0:pi/25:3*pi;
    r = 1;
    num_points = size(theta,2);
    cos_x = (1:num_points)./num_points;
    cos_y = r.*cos(theta);
    
    %% Plot
    f = figure(figureCounter);
    figureCounter = figureCounter + 1;
    subplot(1,3,1);
    plot(line_x, line_y, 'LineWidth', 2);
    title("Line with Elbow")
    set(gca, 'FontName', 'Times');

    subplot(1,3,2);
    plot(circle_x, circle_y, 'LineWidth', 2);
    title("Circle")
    set(gca, 'FontName', 'Times');

    subplot(1,3,3);
    plot(cos_x, cos_y, 'LineWidth', 2);
    title("Sinusoid")
    set(gca, 'FontName', 'Times');
    
    f.Position = [200 200 775 200];
end