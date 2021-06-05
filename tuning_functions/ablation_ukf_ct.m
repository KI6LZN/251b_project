% function to perform an ablation study for constant turn motion model
% for Unscented Kalman filter

function ablation_ukf_ct
    %% Prepare workspace
    close all;
    close(findall(0,'type','figure'));
    clear;
    clc;
    figureCounter = 1;
    
    rng(3);
    noise_amp = 0.1; %tuned values are 0 or 0.1
    noise_corr = noise_amp/2;
    
    %% run tests
    [ln_err, figureCounter] = run_line(figureCounter, noise_amp, noise_corr);
    [circ_err, figureCounter] = run_circle(figureCounter, noise_amp, noise_corr);
    [sin_err, figureCounter] = run_sin(figureCounter, noise_amp, noise_corr);
    
    %% output mean errors
    m_err = [mean(ln_err), mean(circ_err), mean(sin_err)]
    
end

%% Calculate Euclidean distance between two (x,y) column vectors
function dist = euclidean_distance(A, B)
    dist = sqrt((A(:,1)-B(:,1)).^2 + (A(:,2)-B(:,2)).^2 );
end

%% Run filter on a line
function [err, figureCounter] = run_line(figureCounter, noise_amp, noise_corr)

    %% generate a line with a slope of 1
    line_x = 0:.5:10;
    line_y = line_x;
    line_x = [line_x, 10.5:.5:20]./20;
    line_y = [line_y, 11:2:50]./20;
    
    num_skip = 1;
    num_points = floor(size(line_x,2)/num_skip);
    
    %% Prepare filter
    noise_flag = (noise_amp ~= 0);
    first_point = [line_x(1); line_y(1)];
    [filt] = generate_filters_ln(first_point, noise_flag);
    measurement = zeros(num_points,2);
    prediction = zeros(num_points,5);
    err = zeros(num_points,1);
    
    %% Run filter
    for iter = 1:num_points
        noise_x = noise_amp.*randn() ;%- noise_corr;
        noise_y = noise_amp.*randn() ;%- noise_corr;
        prediction(iter,:) = (predict(filt)).';
        
        gnd = [line_x(iter*num_skip); line_y(iter*num_skip)];
        centroid = gnd + [noise_x;noise_y];
        measurement(iter, :) = centroid;
        
        centroid = [centroid; 0];
        
        err(iter) = euclidean_distance([prediction(iter,1), prediction(iter,3)], gnd.');
        correct(filt, centroid);
    end
    
    meas_err = euclidean_distance(measurement, [line_x(1:num_skip:end).' line_y(1:num_skip:end).']);

    %% plot
    figure(figureCounter)
    figureCounter = figureCounter+1;
    title("Line")
    hold on
    plot(prediction(:,1),prediction(:,3), '--o')
    plot(line_x,line_y, '-')
    plot(measurement(:,1), measurement(:,2), '^')
    legend("UKF - CT", "Gnd Truth", "Measured");
    hold off
    
    figure(figureCounter)
    figureCounter = figureCounter+1;
    title("Line Error")
    hold on
    plot(err)
    plot(meas_err)
    legend("UKF - CT", "Measured Err");
    hold off
end

%% Run filter on a circle
function [err, figureCounter] = run_circle(figureCounter, noise_amp, noise_corr)

    %% generate a circle with a radius of 1
    theta = 0:pi/25:2*pi;
    r = 1;
    circle_x = r.*cos(theta);
    circle_y = r.*sin(theta);
    num_skip = 1;
    num_points = floor(size(theta,2)/num_skip);
    measurement = zeros(num_points, 2);

    %% prepare filter
    noise_flag = (noise_amp ~= 0);
    first_point = [circle_x(1); circle_y(1)];
    [filt] = generate_filters_cir(first_point, noise_flag);
    prediction = zeros(num_points,5);
    err = zeros(num_points,1);
    
    %% run filter
    for iter = 1:num_points
        noise_x = noise_amp.*randn() ;%- noise_corr;
        noise_y = noise_amp.*randn() ;%- noise_corr;
        prediction(iter,:) = (predict(filt)).';
        
        gnd = [circle_x(iter*num_skip); circle_y(iter*num_skip)];
        centroid = gnd + [noise_x;noise_y];
        measurement(iter ,:) = centroid;
        
        centroid = [centroid; 0];
        err(iter) = euclidean_distance([prediction(iter,1), prediction(iter,3)], gnd.');
        correct(filt, centroid);
    end
    
    meas_err = euclidean_distance(measurement, [circle_x(1:num_skip:end).' circle_y(1:num_skip:end).']);
    
    %% plot
    figure(figureCounter)
    figureCounter = figureCounter+1;
    title("Circle")
    hold on
    plot(prediction(:,1),prediction(:,3), '--o')
    plot(circle_x,circle_y, '-')
    plot(measurement(:,1), measurement(:,2), '^')
    legend("UKF - CT", "Gnd Truth", "Measured");
    hold off
    
    figure(figureCounter)
    figureCounter = figureCounter+1;
    title("Circle Error")
    hold on
    plot(err)
    plot(meas_err)
    legend("UKF - CT", "Measured Err");
    hold off
end

%% Run filter on a sinusoid
function [err, figureCounter] = run_sin(figureCounter, noise_amp, noise_corr)
 %% generate a sinusoid with an amplitude of 1
    theta = 0:pi/25:3*pi;
    r = 1;
    num_points = size(theta,2);
    cos_x = (1:num_points)./num_points;
    cos_y = r.*cos(theta);
    num_skip = 1;
    num_points = floor(size(theta,2)/num_skip);
    measurement = zeros(num_points, 2);

    %% prepare filter
    noise_flag = (noise_amp ~= 0);
    first_point = [cos_x(1); cos_y(1)];
    [filt] = generate_filters_cos(first_point, noise_flag);
    prediction = zeros(num_points,5);
    err = zeros(num_points,1);
    
    %% run filter
    for iter = 1:num_points
        noise_x = noise_amp.*randn() ;%- noise_corr;
        noise_y = noise_amp.*randn() ;%- noise_corr;
        prediction(iter,:) = (predict(filt)).';
        
        gnd = [cos_x(iter*num_skip); cos_y(iter*num_skip)];
        centroid = gnd + [noise_x;noise_y];
        measurement(iter ,:) = centroid;
        centroid = [centroid; 0];
        
        err(iter) = euclidean_distance([prediction(iter,1), prediction(iter,3)], gnd.');
        correct(filt, centroid);
    end
    
    meas_err = euclidean_distance(measurement, [cos_x(1:num_skip:end).' cos_y(1:num_skip:end).']);
    
    %% plot
    figure(figureCounter)
    figureCounter = figureCounter+1;
    title("Sinusoid")
    hold on
    plot(prediction(:,1),prediction(:,3), '--o')
    plot(cos_x,cos_y, '-')
    plot(measurement(:,1), measurement(:,2), '^')
    legend("UKF - CT", "Gnd Truth", "Measured");
    hold off
    
    figure(figureCounter)
    figureCounter = figureCounter+1;
    title("Sinusoid Error")
    hold on
    plot(err)
    plot(meas_err)
    legend("UKF - CT", "Measured Err");
    hold off

end

%% Functions to generate filters for different motions
function [ukf_ct] = generate_filters_ln(centroid, noise_flag)
    % Create an Unscented Kalman Filter object
    
    % values for noise amplitude 0.1
    if noise_flag == 1
        pos = 2;
        vel = .1;
        cov = eye(5);
        cov(1,1) = pos;
        cov(2,2) = vel;
        cov(3,3) = pos;
        cov(4,4) = vel;
        cov(5,5) = 1e-3;
        meas = 10;
        alpha = 1e-6;
        beta = 2;
        omega = 0.1;
    else
    % values for no noise
        pos = 1;
        vel = 20;
        cov = eye(5);
        cov(1,1) = pos;
        cov(2,2) = vel;
        cov(3,3) = pos;
        cov(4,4) = vel;
        cov(5,5) = 1e-6;
        meas = 1e-3;
        alpha = 1;
        beta = 2;
        omega = 1e-2;
    end
    
    ukf_ct = trackingUKF(@constturn,@ctmeas,[centroid(1);0;centroid(2);0;omega],...
        'StateCovariance', cov, 'ProcessNoise', cov, ...
        'MeasurementNoise', meas, 'Alpha', alpha, 'Beta', beta);
end

function [ukf_ct] = generate_filters_cir(centroid, noise_flag)
    % Create an Unscented Kalman Filter object
    
    % values for noise amplitude 0.1
    if noise_flag == 1
        pos = 0.1;
        vel = 18;
        cov = eye(5);
        cov(1,1) = pos;
        cov(2,2) = vel;
        cov(3,3) = pos;
        cov(4,4) = vel;
        cov(5,5) = 0.8;
        meas = 1025;
        alpha = 1e-3;
        beta = 2;
        omega = 8;
    else
        % values for no noise
        pos = 1;
        vel = 100;
        cov = eye(5);
        cov(1,1) = pos;
        cov(2,2) = vel;
        cov(3,3) = pos;
        cov(4,4) = vel;
        cov(5,5) = 0.1;
        meas = 1e-3;
        alpha = 1e-3;
        beta = 1e6;
        omega = 7.25;
    end
    ukf_ct = trackingUKF(@constturn,@ctmeas,[centroid(1);0;centroid(2);0;omega],...
        'StateCovariance', cov, 'ProcessNoise', cov, ...
        'MeasurementNoise', meas, 'Alpha', alpha, 'Beta', beta);
end

function [ukf_ct] = generate_filters_cos(centroid, noise_flag)
    % Create an Unscented Kalman Filter object
    
    if noise_flag == 1
    % values for noise amplitude 0.1
        pos = 0.1;
        vel = 15;
        cov = eye(5);
        cov(1,1) = pos;
        cov(2,2) = vel;
        cov(3,3) = pos;
        cov(4,4) = vel;
        cov(5,5) = 1.5;
        meas = 500;
        alpha = 1;
        beta = 2;
        omega = 0.5;
    else
    % values for no noise
        pos = 0.1;
        vel = 150;
        cov = eye(5);
        cov(1,1) = pos;
        cov(2,2) = vel;
        cov(3,3) = pos;
        cov(4,4) = vel;
        cov(5,5) = 1;
        meas = 1e-6;
        alpha = 1e-3;
        beta = 2;
        omega = 1e-2;
    end
    
    ukf_ct = trackingUKF(@constturn,@ctmeas,[centroid(1);0;centroid(2);0;omega],...
        'StateCovariance', cov, 'ProcessNoise', cov, ...
        'MeasurementNoise', meas, 'Alpha', alpha, 'Beta', beta);
end