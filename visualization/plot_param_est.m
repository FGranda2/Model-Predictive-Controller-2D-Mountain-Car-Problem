% plot_param_est: Function for visualizing estimated parameters.
%
% Inputs:
%       data_size:      An array containing lengths of datasets used for
%                       identifying the parameters
%       mu_lr:          An array containing maximum likelihood estimates
%                       for the LR model
%       mu_blr:         An array containing the mean estimates from BLR
%       cov_blr:        A cell array containing the covariance estimates
%                       from BLR
%
% --
% Control for Robotics
% AER1517 Spring 2022
% Assignment 3
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistant: 
% SiQi Zhou
% siqi.zhou@robotics.utias.utoronto.ca
% Lukas Brunke
% lukas.brunke@robotics.utias.utoronto.ca
% Adam Hall
% adam.hall@robotics.utias.utoronto.ca
%
% --
% Revision history
% [20.03.07, SZ]    first version
% [22.03.02, SZ]    second version

function [] = plot_param_est(data_size, mu_lr, mu_blr, cov_blr)
    % Number of cases to plot
    data_cases = data_size(1:size(mu_lr,2));

    % Set colors
    blr_color = 'b';
    lr_color = [0.8500, 0.3250, 0.0980];
    
    % Get standard deviation for BLR
    for i = 1:1:length(cov_blr)
       std_blr(:, i) = sqrt(diag(cov_blr{i})); 
    end
    
    % Plot distributions
    subplot(2, 2, [1,3]); cla; hold on;
    for i = 1:1:length(data_cases)
        if i == length(data_cases)
            [~, h1] = plotcov(cov_blr{i}, mu_blr(:,i)', 'FaceColor', ...
                blr_color, 'FaceAlpha', 0.5, 'EdgeColor', 'None');
            h3 = scatter(mu_lr(1,i), mu_lr(2,i), 'x', 'MarkerEdgeColor', ...
                lr_color, 'LineWidth', 1.5, 'SizeData', 45);
        else
            [~, h2] = plotcov(cov_blr{i}, mu_blr(:,i)', 'FaceColor', [0,0,0], ...
                'FaceAlpha', 0.1, 'EdgeColor', 'None');
        end
    end
    xlabel('$\alpha$', 'interpreter', 'latex');
    ylabel('$\beta$', 'interpreter', 'latex');
    title(sprintf('Parameter Estimates (D = %d)', data_cases(end)));
    if length(data_cases) == 1
        legend([h3, h1], 'LR', 'BLR (3 Std.)', 'location', 'northwest');
    else
        legend([h3, h1, h2], 'LR', 'BLR (3 Std.)', ...
            'Previous BLR (3 Std.)', 'location', 'northwest');
    end
    hold on;
    
    % Setup uncertainty bounds
    num_std = 3;
    data_cases_shade = [data_cases, fliplr(data_cases)];
    bounds_blr = [mu_blr - num_std * std_blr, ...
        fliplr(mu_blr + num_std * std_blr)];
    
    % Convergence plot
    subplot(2,2,2); cla;
    semilogx(data_cases, mu_lr(1,:), 'color', lr_color); hold on;
    semilogx(data_cases, mu_blr(1,:), 'color', blr_color);
    fill(data_cases_shade, bounds_blr(1, :), '', 'FaceColor', ...
                blr_color, 'FaceAlpha', 0.15, 'EdgeColor', 'None');
	xlabel('Dataset Size', 'interpreter', 'latex');
    ylabel('$\alpha$', 'interpreter', 'latex');
    legend('LR', 'BLR', 'location', 'southeast');
    set(gca, 'YLim', [-0.0050    0.0050]);
    grid on;
    
    subplot(2,2,4); cla;
    semilogx(data_cases, mu_lr(2,:), 'color', lr_color); hold on;
    semilogx(data_cases, mu_blr(2,:), 'color', blr_color);
    fill(data_cases_shade, bounds_blr(2, :), '', 'FaceColor', ...
                blr_color, 'FaceAlpha', 0.15, 'EdgeColor', 'None');
    xlabel('Dataset Size', 'interpreter', 'latex');
    ylabel('$\beta$', 'interpreter', 'latex');
    set(gca, 'YLim', [-0.0050    0.0050]);
    grid on;
end