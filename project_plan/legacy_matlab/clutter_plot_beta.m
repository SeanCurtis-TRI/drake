close all;
clear;

% === Parameters ===
E_list = [1e3, 1e5, 1e7, 1e9, 1e11, 1e13];
%delta_list = [1e-4, 5e-4, 1e-3];
delta_list = [1e-4];
beta_list = [16, 8, 4, 2, 1, 0.1, 0.01, 0.001];
datadir = 'save_accuracy_1e-3/clutter_data_beta';

E_str    = {"1e3", "1e5", "1e7", "1e9", "1e11", "1e13"};
%d_str    = ["1e-4", "5e-4", "1e-3"];
d_str    = {"1e-4"};
beta_str = {"16", "8", "4", "2", "1", "0.1", "0.01", "0.001"};

% Struct to hold results
results = struct();

% --- global defaults for figures ---
set (0, "defaultaxesfontname", "Helvetica");
set (0, "defaultaxesfontsize", 30);     % tick labels
set (0, "defaulttextfontsize", 20);     % titles, axis labels
set (0, "defaultlinelinewidth", 1.5);     % plot line thickness
set (0, "defaultaxeslinewidth", 1.5);   %s axis box thickness

% === Loop over files ===
for Ei = 1:numel(E_list)
    for di = 1:numel(delta_list)
        for bi = 1:numel(beta_list)
            E = E_list(Ei);
            d = delta_list(di);
            beta = beta_list(bi);
            
            % Construct filename
            fname = sprintf('%s_%s/E_%s_d_%s.txt', datadir, beta_str{bi}, E_str{Ei}, d_str{di});

            % Load numeric data (9 columns)
            data = load(fname);

            time               = data(:,1);
            timestep           = data(:,2);
            iterations         = data(:,3);
            max_condition      = data(:,4);
            last_condition     = data(:,5);
            max_e0             = data(:,6);
            mean_e0            = data(:,7);
            max_A0             = data(:,8);
            mean_A0            = data(:,9);
            total_fn0          = data(:,10);
            max_ls_iterations  = data(:,11);
            mean_ls_iterations = data(:,12);

            % Store in struct
            key = sprintf('E_%g_d_%g_beta_%g', E, d, beta);
            results.(key).time = time;
            results.(key).iterations = iterations;
            results.(key).max_condition = max_condition;
            results.(key).last_condition = last_condition;
            results.(key).max_e0 = max_e0;
            results.(key).mean_e0 = mean_e0;
            results.(key).max_A0 = max_A0;
            results.(key).mean_A0 = mean_A0;
            results.(key).total_fn0 = total_fn0;
            results.(key).max_ls_iterations = max_ls_iterations;
            results.(key).mean_ls_iterations = mean_ls_iterations;

            % === Compute averages over file ===
            results.(key).mean_iterations    = mean(iterations);
            results.(key).mean_max_condition = mean(max_condition);
            results.(key).mean_last_condition = mean(last_condition);
            results.(key).mean_max_e0 = mean(max_e0(max_e0 != 0));
            results.(key).mean_mean_e0 = mean(mean_e0(mean_e0 != 0));
            
            % === Compute maxes over file ===
            results.(key).max_max_condition_number = max(max_condition);
            results.(key).max_last_condition_number = max(last_condition);
            results.(key).max_max_e0 = max(max_e0);
            
            % === Compute totals over file ===
            results.(key).total_iterations = sum(iterations);
            
            results.(key).total_timesteps = length(iterations);
        end
    end
end

% === Collect averages into arrays ===
total_iter = zeros(numel(beta_list), numel(E_list));
max_cond = zeros(numel(beta_list), numel(E_list));
mean_cond = zeros(numel(beta_list), numel(E_list));
max_e = zeros(numel(beta_list), numel(E_list));
mean_e = zeros(numel(beta_list), numel(E_list));
mean_max_e = zeros(numel(beta_list), numel(E_list));
total_timesteps = zeros(numel(beta_list), numel(E_list));
average_iterations = zeros(numel(beta_list), numel(E_list));

for Ei = 1:numel(E_list)
    for bi = 1:numel(beta_list)
        di = 1;
        E = E_list(Ei);
        d = delta_list(di);
        beta = beta_list(bi);
        key = sprintf('E_%g_d_%g_beta_%g', E, d, beta);
        total_iter(bi,Ei) = results.(key).total_iterations;
        max_cond(bi, Ei) = results.(key).max_max_condition_number;
        mean_cond(bi, Ei) = results.(key).mean_last_condition;
        mean_e(bi, Ei) = results.(key).mean_mean_e0;
        max_e(bi, Ei) = results.(key).max_max_e0;
        mean_max_e(bi, Ei) = results.(key).mean_max_e0;
        total_timesteps(bi, Ei) = results.(key).total_timesteps;
        average_iterations(bi, Ei) = total_iter(bi, Ei) / total_timesteps(bi, Ei);
    end
end

% === Summary plots across beta ===

% Total iterations vs beta
figure("position", [0, 0, 3840, 2160]); hold on;
for Ei = 1:numel(E_list)
    semilogx(beta_list, total_iter(:,Ei), '-o', 'DisplayName', sprintf('E=%.0e', E_list(Ei)));
end
xlabel('beta'); ylabel('Total iterations');
h = legend('show'); legend('location', 'best', 'box', 'off');
set (h, "fontsize", 20);
title('Total iterations vs beta');
box on;
print(sprintf("beta_total_iterations.png", datadir), "-S1920,1080", "-r600", "-dpng")

% Max condition vs beta
figure("position", [0, 0, 3840, 2160]); hold on;
for Ei = 1:numel(E_list)
    loglog(beta_list, max_cond(:,Ei), '-o', 'DisplayName', sprintf('E=%.0e', E_list(Ei)));
end
xlabel('beta'); ylabel('Max condition number');
h = legend('show'); legend('location', 'best', 'box', 'off');
set (h, "fontsize", 20);
title('Max condition number vs beta');
box on;
print(sprintf("beta_max_condition.png", datadir), "-S1920,1080", "-r600", "-dpng")


% Mean condition vs beta
figure("position", [0, 0, 3840, 2160]); hold on;
for Ei = 1:numel(E_list)
    loglog(beta_list, mean_cond(:,Ei), '-o', 'DisplayName', sprintf('E=%.0e', E_list(Ei)));
end
xlabel('beta'); ylabel('Mean condition number');
h = legend('show'); legend('location', 'best', 'box', 'off');
set (h, "fontsize", 20);
title('Mean condition number vs beta');
box on;
print(sprintf("beta_mean_condition.png", datadir), "-S1920,1080", "-r600", "-dpng")


% Max e vs beta
figure("position", [0, 0, 3840, 2160]); hold on;
for Ei = 1:numel(E_list)
    semilogx(beta_list, mean_max_e(:,Ei), '-o', 'DisplayName', sprintf('E=%.0e', E_list(Ei)));
end

xlabel('beta'); ylabel('Mean extent');
h = legend('show'); legend('location', 'best', 'box', 'off');
set (h, "fontsize", 20);
title('Mean of max extent vs beta');
box on;
print(sprintf("beta_max_e.png", datadir), "-S1920,1080", "-r600", "-dpng")

%figure();
%b2e = [1e7*beta_list.^2, 1e8* beta_list.^2, 1e9 * beta_list.^2, 1e10 * beta_list.^2];
%e2 = [mean_max_e(:,1)', mean_max_e(:,2)', mean_max_e(:,3)', mean_max_e(:,4)'];
%semilogx(b2e, e2, '-o');


% Mean max e vs beta
figure("position", [0, 0, 3840, 2160]); hold on;
for Ei = 1:numel(E_list)
    semilogx(beta_list, mean_e(:,Ei), '-o', 'DisplayName', sprintf('E=%.0e', E_list(Ei)));
end
xlabel('beta'); ylabel('Mean extent');
h = legend('show'); legend('location', 'best', 'box', 'off');
set (h, "fontsize", 20);
title('Mean extent vs beta');
box on;
print(sprintf("beta_mean_extent.png", datadir), "-S1920,1080", "-r600", "-dpng")

figure("position", [0, 0, 3840, 2160]); hold on;
for Ei = 1:numel(E_list)
    semilogx(beta_list, total_timesteps(:,Ei), '-o', 'DisplayName', sprintf('E=%.0e', E_list(Ei)));
end
xlabel('beta'); ylabel('Total timestpes');
h = legend('show'); legend('location', 'best', 'box', 'off');
set (h, "fontsize", 20);
title('Total timesteps vs beta');
box on;
print(sprintf("beta_total_timesteps.png", datadir), "-S1920,1080", "-r600", "-dpng")

figure("position", [0, 0, 3840, 2160]); hold on;
for Ei = 1:numel(E_list)
    semilogx(beta_list, average_iterations(:,Ei), '-o', 'DisplayName', sprintf('E=%.0e', E_list(Ei)));
end
xlabel('beta'); ylabel('Average iterations');
h = legend('show'); legend('location', 'best', 'box', 'off');
set (h, "fontsize", 20);
title('Average iterations vs beta');
box on;
print(sprintf("beta_average_iterations.png", datadir), "-S1920,1080", "-r600", "-dpng")

