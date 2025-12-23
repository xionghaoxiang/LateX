clc; clear; close all;

%% 1. Data Input
% Format: {DroneID, BombID, TargetID, StartTime, EndTime}
taskDataRaw = {
    'FY1', '#1', 'M1',  3.00,  6.80;
    'FY1', '#2', 'M1',  4.60,  8.40;
    'FY2', '#1', 'M2',  9.00, 13.00;
    'FY2', '#2', 'M1', 24.80, 25.60;
    'FY3', '#2', 'M3', 35.60, 38.60; 
    'FY3', '#1', 'M2', 37.80, 41.20;
    'FY3', '#3', 'M1', 49.40, 51.00;
    'FY4', '#1', 'M2', 15.00, 19.40;
    'FY5', '#1', 'M1', 19.60, 23.60;
};

% Sort by Drone ID and Start Time to ensure correct stacking logic
[~, idx] = sortrows(taskDataRaw, [1, 4]);
taskData = taskDataRaw(idx, :);

%% 2. Plot Settings
droneNames = {'FY5', 'FY4', 'FY3', 'FY2', 'FY1'}; % Y-axis from bottom to top
n_drones = length(droneNames);

% Define Colors for Targets
targetColors = containers.Map();
targetColors('M1') = [0.85, 0.33, 0.10]; % Orange-Red
targetColors('M2') = [0.00, 0.45, 0.74]; % Blue
targetColors('M3') = [0.47, 0.67, 0.19]; % Green

barHeight = 0.35;    % Height of a single bar
levelSpacing = 0.45; % Vertical spacing between overlapping tasks

figure('Color', 'w', 'Position', [100, 100, 1200, 700]);
hold on; grid on; box on;

h_legend = [];
legend_labels = {};

%% 3. Plotting with Overlap Handling
% Map to track the end time of each level for each drone
droneLevels = containers.Map(); 
for i = 1:n_drones
    droneLevels(droneNames{i}) = []; 
end

for i = 1:size(taskData, 1)
    droneID = taskData{i, 1};
    bombID = taskData{i, 2};
    targetID = taskData{i, 3};
    t_start = taskData{i, 4};
    t_end = taskData{i, 5};
    
    base_y_idx = find(strcmp(droneNames, droneID));
    
    % --- Level Calculation Logic ---
    levels = droneLevels(droneID);
    task_level = 0;
    foundLevel = false;
    
    % Try to find a level where this task fits without overlapping
    for lvl = 1:length(levels)
        if t_start >= levels(lvl) 
            task_level = lvl - 1; 
            levels(lvl) = t_end; 
            foundLevel = true;
            break;
        end
    end
    
    % Create a new level if needed
    if ~foundLevel
        task_level = length(levels);
        levels = [levels, t_end];
    end
    droneLevels(droneID) = levels;
    % -------------------------------
    
    % Calculate Y position
    y_center = base_y_idx + task_level * levelSpacing - (length(levels)-1)*levelSpacing/2;
    
    % Get Color
    if isKey(targetColors, targetID)
        c = targetColors(targetID);
    else
        c = [0.5 0.5 0.5];
    end
    
    % Draw Patch
    x_patch = [t_start, t_end, t_end, t_start];
    y_low = y_center - barHeight/2;
    y_high = y_center + barHeight/2;
    y_patch = [y_low, y_low, y_high, y_high];
    
    h = patch(x_patch, y_patch, c, 'EdgeColor', 'k', 'FaceAlpha', 0.8, 'LineWidth', 0.5);
    
    % Handle Legend (Unique entries only)
    if ~ismember(targetID, legend_labels)
        h_legend = [h_legend, h]; %#ok<AGROW>
        legend_labels = [legend_labels, targetID]; %#ok<AGROW>
    end
    
    % Text: Bomb ID (Inside bar)
    mid_x = (t_start + t_end) / 2;
    text(mid_x, y_center, bombID, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontSize', 8, 'FontWeight', 'bold', 'Color', 'w');
        
    % Text: Time Interval (Outside bar, alternating position)
    label_y_pos = y_high + 0.1;
    if mod(task_level, 2) ~= 0
        label_y_pos = label_y_pos + 0.15;
    end
    
    text(mid_x, label_y_pos, sprintf('[%.1f-%.1f]', t_start, t_end), ...
        'HorizontalAlignment', 'center', 'FontSize', 7.5, 'Color', [0.2 0.2 0.2], 'Interpreter', 'none');
end

%% 4. Axes and Labels (English)
set(gca, 'YTick', 1:n_drones, 'YTickLabel', droneNames, 'FontSize', 11);
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Drone ID', 'FontSize', 12, 'FontWeight', 'bold');

% Title in English
title('Smoke Screening Effective Time Gantt Chart (Total: 26.60s)', 'FontSize', 14);

max_time = max([taskData{:, 5}]);
xlim([0, max_time + 5]);
ylim([0.2, n_drones + 1.5]); 

% Legend in English (Sorted)
[~, sort_idx] = sort(legend_labels);
% Assuming targets are named M1, M2, etc. If you want "Target M1", modify below:
legend(h_legend(sort_idx), legend_labels(sort_idx), ...
    'Location', 'NorthEast', 'Orientation', 'horizontal', 'FontSize', 10, 'Box', 'on');

hold off;