close all;
clear;

datadir = 'clutter_data';

% === Fixed parameters ===
E = 1e7;
beta = 1;
delta_list = [0, 1e-5, 1e-4, 5e-4, 1e-3];
delta_str = {"0", "1e-5", "1e-4", "5e-4", "1e-3"};
accuracy_list = [1e-1, 1e-2, 1e-3];
accuracy_str = {"1e-1", "1e-2", "1e-3"};

% --- global defaults for figures ---
set (0, "defaultaxesfontname", "Helvetica");
set (0, "defaultaxesfontsize", 10);
set (0, "defaulttextfontsize", 10);
set (0, "defaultlinelinewidth", 1);
set (0, "defaultaxeslinewidth", 1);

colors = lines(numel(delta_list) + 1);

% === Initialize figures ===
fig_iters = figure("position", [0, 0, 3840, 2160]);
fig_steps = figure("position", [0, 0, 3840, 2160]);
fig_cond_max = figure("position", [0, 0, 3840, 2160]);
fig_cond_mean = figure("position", [0, 0, 3840, 2160]);
fig_e_max = figure("position", [0, 0, 3840, 2160]);
fig_e_mean = figure("position", [0, 0, 3840, 2160]);
fig_step_ratio = figure("position", [0, 0, 3840, 2160]);

% === New: figures for num_constraints vs time (one per accuracy) ===
fig_nc_time = gobjects(numel(accuracy_list), 1);
for ai = 1:numel(accuracy_list)
    fig_nc_time(ai) = figure("position", [0, 0, 3840, 2160]);
end

for di = 1:numel(delta_list)
    delta = delta_list(di);
    d_str = delta_str{di};

    total_iterations = zeros(size(accuracy_list));
    total_timesteps  = zeros(size(accuracy_list));
    max_condition    = zeros(size(accuracy_list));
    mean_condition   = zeros(size(accuracy_list));
    max_e            = zeros(size(accuracy_list));
    mean_e           = zeros(size(accuracy_list));
    step_ratio       = zeros(size(accuracy_list));

    for ai = 1:numel(accuracy_list)
        acc_str = accuracy_str{ai};
        fname_full     = sprintf('%s/b_%s_ac_%s.txt', datadir, d_str, acc_str);
        fname_accepted = sprintf('%s/b_%s_ac_%s.txt_accepted', datadir, d_str, acc_str);

        if ~isfile(fname_full)
            warning("Missing file: %s", fname_full);
            continue;
        end

        if ~isfile(fname_accepted)
            warning("Missing file: %s", fname_accepted);
            continue;
        end

        data_full     = load(fname_full);
        data_accepted = load(fname_accepted);

        iterations      = data_full(:,3);
        time            = data_accepted(:,1);
        max_condition_  = data_accepted(:,7);
        last_condition  = data_accepted(:,8);
        max_e0          = data_accepted(:,9);
        mean_e0         = data_accepted(:,10);
        num_constraints = data_accepted(:,11);

        total_iterations(ai) = sum(iterations);
        total_timesteps(ai)  = numel(max_e0);
        max_condition(ai)    = max(max_condition_);
        mean_condition(ai)   = mean(last_condition);
        max_e(ai)            = max(max_e0(max_e0 > 0));
        mean_e(ai)           = mean(mean_e0);
        step_ratio(ai)       = (length(data_full) - length(data_accepted)) / length(data_accepted);

        % === New: plot num_constraints vs time for this accuracy, curve labeled by delta ===
        figure(fig_nc_time(ai));
        plot(time, num_constraints, '-', ...
             'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
        hold on;
        grid on; box on;
    end

    % === Existing plots for this delta ===
    figure(fig_iters);
    semilogx(accuracy_list, total_iterations, '-o', 'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_steps);
    semilogx(accuracy_list, total_timesteps, '-o', 'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_cond_max);
    loglog(accuracy_list, max_condition, '-o', 'Color', colors(di,:), ...
           'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_cond_mean);
    loglog(accuracy_list, mean_condition, '-o', 'Color', colors(di,:), ...
           'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_e_max);
    semilogx(accuracy_list, max_e, '-o', 'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_e_mean);
    semilogx(accuracy_list, mean_e, '-o', 'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_step_ratio);
    semilogx(accuracy_list, step_ratio, '-o', 'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;
end

% === Finalize existing figures ===
figure(fig_iters);
xlabel('Accuracy'); ylabel('Total iterations');
title('Total iterations vs accuracy');
legend('show','Location','best','Box','off');
grid on; box on;
exportgraphics(fig_iters, 'accuracy_total_iterations.png', 'Resolution', 600);

figure(fig_steps);
xlabel('Accuracy'); ylabel('Number of steps');
title('Number of steps vs accuracy');
legend('show','Location','best','Box','off');
grid on; box on;
exportgraphics(fig_steps, 'accuracy_num_steps.png', 'Resolution', 600);

figure(fig_cond_max);
xlabel('Accuracy'); ylabel('Max condition number');
title('Max condition number vs accuracy');
legend('show','Location','best','Box','off');
grid on; box on;
exportgraphics(fig_cond_max, 'accuracy_condition_max.png', 'Resolution', 600);

figure(fig_cond_mean);
xlabel('Accuracy'); ylabel('Mean condition number');
title('Average last iteration condition number vs accuracy');
legend('show','Location','best','Box','off');
grid on; box on;
exportgraphics(fig_cond_mean, 'accuracy_condition_mean.png', 'Resolution', 600);

figure(fig_e_max);
xlabel('Accuracy'); ylabel('Max e');
title('Max extent vs accuracy');
legend('show','Location','best','Box','off');
grid on; box on;
exportgraphics(fig_e_max, 'accuracy_e_max.png', 'Resolution', 600);

figure(fig_e_mean);
xlabel('Accuracy'); ylabel('Mean e');
title('Mean extent vs accuracy');
legend('show','Location','best','Box','off');
grid on; box on;
exportgraphics(fig_e_mean, 'accuracy_e_mean.png', 'Resolution', 600);

figure(fig_step_ratio);
xlabel('Accuracy'); ylabel('Ratio of failed steps');
title('Failed step ratio vs accuracy');
legend('show','Location','best','Box','off');
grid on; box on;
exportgraphics(fig_step_ratio, 'accuracy_step_ratio.png', 'Resolution', 600);

% === Finalize new num_constraints vs time figures ===
for ai = 1:numel(accuracy_list)
    figure(fig_nc_time(ai));
    xlabel('Time');
    ylabel('Number of constraints');
    title(sprintf('Constraints vs time (accuracy = %s)', accuracy_str{ai}));
    legend('show','Location','best','Box','off');
    grid on; box on;
    fname = sprintf('constraints_vs_time_ac_%s.png', accuracy_str{ai});
    exportgraphics(fig_nc_time(ai), fname, 'Resolution', 600);
end

% Wall clocks from the most recent run:
% wall_clock_b_0 = [10.929194908, 11.738351328, 20.211941663];
% wall_clock_b_1e_5 = [56.816555429, 70.941694511, 117.924355142];
% wall_clock_b_1e_4 = [43.685574692, 57.789414557, 94.105615344];
% wall_clock_b_5e_4 = [35.74469016, 50.304710359, 80.926851255];
% wall_clock_b_1e_3 = [34.985361847, 48.555949025, 77.982134741];

% wall_clock_b_0 = [2.195374183, 4.372962057, 6.231106721];
% wall_clock_b_1e_5 = [42.311183392, 32.938869347, 46.304363121];
% wall_clock_b_1e_4 = [20.703672339, 24.826082545, 37.156262526];
% wall_clock_b_5e_4 = [14.558552149, 11.555916081, 23.745563666];
% wall_clock_b_1e_3 = [9.905018259, 10.874382538, 20.692738027];

% Wall clock from E 1e9 margin = 0

% wall_clock_b_0 = [5.282552638, 8.848814677, 14.489065826];
% wall_clock_b_1e_3 = [44.19123127, 43.389742064, 40.876730126];
% wall_clock_b_1e_4 = [60.452071381, 67.674455317, 63.50752021];
% wall_clock_b_1e_5 = [104.437574347, 124.333059014, 117.622679277];
% wall_clock_b_5e_4 = [32.572521055, 35.12410077, 38.750104935]

% Wall clock from E 1e9 margin = 1e-4

wall_clock_b_0 = [3.912256411, 5.438095645, 11.704304193];
wall_clock_b_1e_3 = [14.822030032, 15.998392146, 33.055036577];
wall_clock_b_1e_4 = [23.899870605, 23.816690481, 49.590758529];
wall_clock_b_1e_5 = [34.523656052, 35.349681678, 50.690990452];
wall_clock_b_5e_4 = [16.484339517, 18.633770307, 46.523445162]

fig_wallclock = figure("position", [0, 0, 3840, 2160]);
semilogx(accuracy_list, wall_clock_b_0, '-o', 'Color', colors(1,:), 'DisplayName', sprintf('\\delta = %s', "0"));
hold on;
semilogx(accuracy_list, wall_clock_b_1e_5, '-o', 'Color', colors(2,:), 'DisplayName', sprintf('\\delta = %s', "1e-5"));
hold on;
semilogx(accuracy_list, wall_clock_b_1e_4, '-o', 'Color', colors(3,:), 'DisplayName', sprintf('\\delta = %s', "1e-4"));
hold on;
semilogx(accuracy_list, wall_clock_b_5e_4, '-o', 'Color', colors(4,:), 'DisplayName', sprintf('\\delta = %s', "5e-4"));
hold on;
semilogx(accuracy_list, wall_clock_b_1e_3, '-o', 'Color', colors(5,:), 'DisplayName', sprintf('\\delta = %s', "1e-3"));
hold on;
figure(fig_wallclock);
xlabel('Accuracy'); ylabel('Wall clock [s]');
title('Wall clock vs accuracy');
legend('show','Location','best','Box','off');
grid on; box on;
exportgraphics(fig_wallclock, 'accuracy_wall_clock.png', 'Resolution', 600);
