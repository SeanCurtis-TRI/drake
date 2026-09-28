% Define files and labels
files = {
    'data/surface_barrier_1e-3_margin_1e-2.txt'
    'data/surface_barrier_1e-3_margin_1e-3.txt'
    'data/surface_barrier_1e-4_margin_1e-3.txt'
    'data/surface_barrier_1e-4_margin_1e-4.txt'
    'data/volumetric.txt'
};

labels = {
    '1e-3 barrier, 1e-2 margin'
    '1e-3 barrier, 1e-3 margin'
    '1e-4 barrier, 1e-3 margin'
    '1e-4 barrier, 1e-4 margin'
    'Volumetric'
};

fixed_step_wall_clock = 1.15848;
fixed_step_num_steps = 401;
fixed_step_num_solves = 401;

colors = lines(numel(files)); % generate distinct colors

% Data storage
data = cell(numel(files), 1);

% Read data
for i = 1:numel(files)
    data{i} = readmatrix(files{i});
    % Expected format per file:
    % [accuracy, wall_clock, num_steps, num_solves]
end

% Extract columns
accuracy   = cellfun(@(d) d(:,1), data, 'UniformOutput', false);
wall_clock = cellfun(@(d) d(:,2), data, 'UniformOutput', false);
num_steps  = cellfun(@(d) d(:,3), data, 'UniformOutput', false);
num_solves = cellfun(@(d) d(:,4), data, 'UniformOutput', false);

%% Plot 1: Accuracy vs Wall Clock
figure;
hold on;
for i = 1:numel(files)
    plot(accuracy{i}, wall_clock{i}, '-o', 'Color', colors(i,:), 'DisplayName', labels{i});
end
yline(fixed_step_wall_clock, 'k--', 'DisplayName', sprintf('Fixed-step = %.5g s', fixed_step_wall_clock), 'LineWidth', 1.2);
set(gca, 'XScale', 'log');
xlabel('Accuracy');
ylabel('Wall clock time [s]');
title('Accuracy vs Wall Clock Time');
legend('Location','best');
grid on;

%% Plot 2: Accuracy vs Number of Steps
figure;
hold on;
for i = 1:numel(files)
    plot(accuracy{i}, num_steps{i}, '-o', 'Color', colors(i,:), 'DisplayName', labels{i});
end
yline(fixed_step_num_steps, 'k--', 'DisplayName', sprintf('Fixed-step = %d steps', fixed_step_num_steps), 'LineWidth', 1.2);
set(gca, 'XScale', 'log');
xlabel('Accuracy');
ylabel('Number of steps');
title('Accuracy vs Number of Steps');
legend('Location','best');
grid on;

%% Plot 3: Accuracy vs Number of Solves
figure;
hold on;
for i = 1:numel(files)
    plot(accuracy{i}, num_solves{i}, '-o', 'Color', colors(i,:), 'DisplayName', labels{i});
end
yline(fixed_step_num_solves, 'k--', 'DisplayName', sprintf('Fixed-step = %d solves', fixed_step_num_solves), 'LineWidth', 1.2);
set(gca, 'XScale', 'log');
xlabel('Accuracy');
ylabel('Number of convex solves');
title('Accuracy vs Number of Solves');
legend('Location','best');
grid on;
